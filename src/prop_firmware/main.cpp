#include <Arduino.h>

// Single firmware for all sticks/diabolos.
// ONLY change wifi_configs.h (DEVICE_NAME) for each physical device.

#include <CodeCell.h>
#include "wifi_configs.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <math.h>

CodeCell myCodeCell;
WiFiUDP Udp;
Preferences prefs;

const char* ssid = WIFI_NAME;
const char* password = WIFI_PASS;
const char* pc_ip = WIFI_IP;
const int port = 9999;

// Tags built once, based on DEVICE_NAME
String tagInertia, tagBat, tagProx, tagState, tagAct, tagGyro, tagComp, tagRot, tagSteps;
String tagLight, tagGrav, tagLin, tagSleep, tagTimeout;

// Incoming command tags: "<DEVICE_NAME>_x" targets this board, "all_x" targets every board.
String cmdSleepMine, cmdSleepAll, cmdTimeoutMine, cmdTimeoutAll;

float ax, ay, az;
float filtered_magnitude = 0;

// --- INERTIA SENSITIVITY (tune for stick vs diabolo if needed) ---
float smoothing_factor = 0.05;
float min_reading = 9.8;
float max_reading = 20.0;
int lastInertia = -1;

// --- PROXIMITY ---
const uint16_t proxMax = 1000;
int lastProx = -1;

// --- STATE (On table / Stationary / Stable / Motion) ---
int lastState = -1;

// --- ACTIVITY (Driving/Cycling/Walking/Still/Tilting/Running/Climbing) ---
int lastAct = -1;

// --- GYROSCOPE (magnitude) ---
const float gyroMax = 250.0;
int lastGyro = -1;

// --- MAGNETOMETER (magnitude, used as "compass") ---
const float compMax = 100.0;
int lastComp = -1;

// --- ROTATION (yaw) ---
int lastRot = -1;

// --- STEPS ---
uint16_t lastSteps = 0;

// --- AMBIENT LIGHT ---
const uint16_t lightMax = 1000;
int lastLight = -1;

// --- GRAVITY (magnitude, should be ~9.8 always, changes with orientation) ---
const float gravMax = 12.0;
int lastGrav = -1;

// --- LINEAR ACCELERATION (magnitude, "net motion" without gravity) ---
const float linMax = 15.0;
int lastLin = -1;

// --- BATTERY ---
unsigned long lastBatteryTime = 0;
const unsigned long batteryInterval = 5000;
uint16_t lastBatLevel = 100;  // 101 = charging, 102 = USB only

// --- AUTO-SLEEP -------------------------------------------------------------
// After this many minutes without movement the board deep-sleeps. Tap the
// pendulum to wake it. 0 = never auto-sleep (use that during a performance).
//
// This is only the FACTORY DEFAULT, used the first time a board ever boots.
// After that the value lives in flash and is changed from Ableton with
// "all_timeout <minutes>" - no reflashing. See handleIncomingUDP().
#define DEFAULT_IDLE_TIMEOUT_MINUTES 10

// How much movement counts as "still being used". Raise these if a hanging,
// perfectly still pendulum never sleeps; lower them if it sleeps mid-swing.
const float motionThresholdLin = 0.5;    // m/s^2, gravity already removed
const float motionThresholdGyro = 10.0;  // deg/s

uint16_t idleTimeoutMinutes = DEFAULT_IDLE_TIMEOUT_MINUTES;
unsigned long lastMotionTime = 0;

void sendUDP(const String &tag, long value) {
  Udp.beginPacket(pc_ip, port);
  Udp.print(tag);
  Udp.print(" ");
  Udp.print(value);
  Udp.println(";");
  Udp.endPacket();
}

// Deep sleep. The Wi-Fi radio is off while asleep, so nothing can wake this
// board over the network - the only way back is a tap on the pendulum, which
// the IMU detects on its own while the rest of the chip is powered down.
void goToSleep() {
  sendUDP(tagSleep, 1);  // tell Max this device is going down on purpose
  Serial.println(">> Sleeping. Tap the pendulum to wake it up.");
  Serial.flush();
  delay(50);  // let the last packet leave before the radio dies

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  myCodeCell.SleepTapTrigger();  // never returns; wakes as a fresh boot
}

// Store the idle timeout in flash so it survives sleeping, reboots and
// reflashing. This is what lets you retune the timeout from Ableton once
// instead of reflashing every board.
void setIdleTimeout(long minutes) {
  if (minutes < 0) minutes = 0;
  if (minutes > 600) minutes = 600;  // 10 h, sanity clamp

  idleTimeoutMinutes = (uint16_t)minutes;
  prefs.putUShort("timeout", idleTimeoutMinutes);
  lastMotionTime = millis();  // restart the countdown under the new value

  Serial.print(">> Idle timeout set to ");
  Serial.print(idleTimeoutMinutes);
  Serial.println(idleTimeoutMinutes == 0 ? " min (auto-sleep disabled)" : " min");

  sendUDP(tagTimeout, idleTimeoutMinutes);  // confirm back to Max
}

// Commands from Ableton:
//   all_sleep 1;        every board sleeps now
//   stickB_sleep 1;     only that board sleeps
//   all_timeout 10;     auto-sleep after 10 min (0 = never)
//
// Two wire formats are accepted, because Max's [udpsend] does NOT send plain
// text - it wraps messages in OSC framing - while netcat and friends do:
//   plain text : "all_sleep 1;"
//   OSC        : "/all_sleep\0\0" + ",i\0\0" + <int32, big-endian>
// Accepting both means the same firmware works from Max and from a terminal,
// which matters because a wrong guess here would fail silently.
void handleIncomingUDP() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;

  uint8_t buf[128];
  int len = Udp.read(buf, sizeof(buf));
  if (len <= 0) return;

  // ---- the tag: first run of printable characters, minus any leading '/' ----
  int i = 0;
  while (i < len && (buf[i] == '/' || buf[i] <= ' ')) i++;

  String tag = "";
  while (i < len && buf[i] > ' ' && buf[i] != ';') {
    tag += (char)buf[i];
    i++;
  }
  if (tag.length() == 0) return;

  // ---- the value, plain text: a number right after the tag ----
  long value = 0;
  bool haveValue = false;

  while (i < len && (buf[i] == ' ' || buf[i] == '\t')) i++;
  if (i < len && (isdigit(buf[i]) || buf[i] == '-')) {
    String num = "";
    while (i < len && (isdigit(buf[i]) || buf[i] == '-')) {
      num += (char)buf[i];
      i++;
    }
    value = num.toInt();
    haveValue = true;
  }

  // ---- the value, OSC: find the ",i"/",f" type tag, read the argument ----
  // The type-tag string is padded to a multiple of 4 bytes, and the argument
  // starts right after that padding.
  if (!haveValue) {
    for (int t = 0; t + 1 < len; t++) {
      if (buf[t] != ',') continue;
      int arg = ((t + 2) + 3) & ~3;
      if (arg + 3 >= len) break;

      uint32_t raw = ((uint32_t)buf[arg] << 24) | ((uint32_t)buf[arg + 1] << 16)
                   | ((uint32_t)buf[arg + 2] << 8) | (uint32_t)buf[arg + 3];
      if (buf[t + 1] == 'i') {
        value = (int32_t)raw;
        haveValue = true;
      } else if (buf[t + 1] == 'f') {
        float f;
        memcpy(&f, &raw, sizeof(f));
        value = (long)f;
        haveValue = true;
      }
      break;
    }
  }

  if (tag == cmdSleepMine || tag == cmdSleepAll) {
    // A bare "all_sleep" with no argument counts as "yes, sleep".
    if (!haveValue || value != 0) goToSleep();
  } else if (tag == cmdTimeoutMine || tag == cmdTimeoutAll) {
    if (haveValue) setIdleTimeout(value);
  }
}

void setup() {
  Serial.begin(115200);

  // NOTE: these suffixes are part of the UDP protocol.
  // If you change them, update the receiver (PC side) too.
  tagInertia = String(DEVICE_NAME) + "_inertia";
  tagBat     = String(DEVICE_NAME) + "_bat";
  tagProx    = String(DEVICE_NAME) + "_prox";
  tagState   = String(DEVICE_NAME) + "_state";
  tagAct     = String(DEVICE_NAME) + "_act";
  tagGyro    = String(DEVICE_NAME) + "_gyro";
  tagComp    = String(DEVICE_NAME) + "_comp";
  tagRot     = String(DEVICE_NAME) + "_rot";
  tagSteps   = String(DEVICE_NAME) + "_steps";
  tagLight   = String(DEVICE_NAME) + "_light";
  tagGrav    = String(DEVICE_NAME) + "_grav";
  tagLin     = String(DEVICE_NAME) + "_lin";
  tagSleep   = String(DEVICE_NAME) + "_sleep";
  tagTimeout = String(DEVICE_NAME) + "_timeout";

  cmdSleepMine   = String(DEVICE_NAME) + "_sleep";
  cmdSleepAll    = "all_sleep";
  cmdTimeoutMine = String(DEVICE_NAME) + "_timeout";
  cmdTimeoutAll  = "all_timeout";

  // Read the idle timeout saved in flash, falling back to the compiled default
  // the very first time this board boots.
  prefs.begin("osc", false);
  idleTimeoutMinutes = prefs.getUShort("timeout", DEFAULT_IDLE_TIMEOUT_MINUTES);

  myCodeCell.Init(LIGHT + MOTION_ACCELEROMETER + MOTION_STATE
                  + MOTION_GYRO + MOTION_MAGNETOMETER + MOTION_ROTATION
                  + MOTION_STEP_COUNTER + MOTION_ACTIVITY
                  + MOTION_GRAVITY + MOTION_LINEAR_ACC);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("Device: "); Serial.println(DEVICE_NAME);

  Udp.begin(port);
  filtered_magnitude = min_reading;

  // Waking from deep sleep is a full reboot, so this doubles as "I am awake
  // again". Without it a sleep indicator in Max would latch on forever.
  sendUDP(tagSleep, 0);
  sendUDP(tagTimeout, idleTimeoutMinutes);

  Serial.print("Auto-sleep: ");
  if (idleTimeoutMinutes == 0) {
    Serial.println("disabled");
  } else {
    Serial.print(idleTimeoutMinutes); Serial.println(" min without movement");
  }

  lastMotionTime = millis();  // start the countdown now, not at time zero
}

void loop() {
  if (myCodeCell.Run(50)) {

    // ---- INERTIA (accelerometer, smoothed magnitude) ----
    myCodeCell.Motion_AccelerometerRead(ax, ay, az);
    float current_magnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
    filtered_magnitude = (current_magnitude * smoothing_factor) + (filtered_magnitude * (1.0 - smoothing_factor));

    int inertiaValue = map((long)(filtered_magnitude * 10), (long)(min_reading * 10), (long)(max_reading * 10), 0, 127);
    inertiaValue = constrain(inertiaValue, 0, 127);
    if (inertiaValue != lastInertia) {
      sendUDP(tagInertia, inertiaValue);
      lastInertia = inertiaValue;
    }

    // ---- PROXIMITY ----
    uint16_t proxRaw = myCodeCell.Light_ProximityRead();
    int proxValue = constrain((int)map(proxRaw, 0, proxMax, 0, 127), 0, 127);
    if (proxValue != lastProx) {
      sendUDP(tagProx, proxValue);
      lastProx = proxValue;
    }

    // ---- STATE: 1=Table, 2=Stationary, 3=Stable, 4=Motion ----
    int state = myCodeCell.Motion_StateRead();
    if (state != lastState) {
      sendUDP(tagState, state);
      lastState = state;
    }

    // ---- ACTIVITY ----
    int act = myCodeCell.Motion_ActivityRead();
    if (act != lastAct) {
      sendUDP(tagAct, act);
      lastAct = act;
    }

    // ---- GYROSCOPE (magnitude) ----
    float gx, gy, gz;
    myCodeCell.Motion_GyroRead(gx, gy, gz);
    float gyroMag = sqrt((gx * gx) + (gy * gy) + (gz * gz));
    int gyroValue = constrain((int)map((long)gyroMag, 0, (long)gyroMax, 0, 127), 0, 127);
    if (gyroValue != lastGyro) {
      sendUDP(tagGyro, gyroValue);
      lastGyro = gyroValue;
    }

    // ---- MAGNETOMETER (magnitude, used as "compass") ----
    float mx, my, mz;
    myCodeCell.Motion_MagnetometerRead(mx, my, mz);
    float compMag = sqrt((mx * mx) + (my * my) + (mz * mz));
    int compValue = constrain((int)map((long)compMag, 0, (long)compMax, 0, 127), 0, 127);
    if (compValue != lastComp) {
      sendUDP(tagComp, compValue);
      lastComp = compValue;
    }

    // ---- ROTATION (yaw, -180..180 -> 0..127) ----
    float roll, pitch, yaw;
    myCodeCell.Motion_RotationRead(roll, pitch, yaw);
    int rotValue = constrain((int)map((long)yaw, -180, 180, 0, 127), 0, 127);
    if (rotValue != lastRot) {
      sendUDP(tagRot, rotValue);
      lastRot = rotValue;
    }

    // ---- STEPS ----
    uint16_t steps = myCodeCell.Motion_StepCounterRead();
    if (steps != lastSteps) {
      sendUDP(tagSteps, (long)steps);
      lastSteps = steps;
    }

    // ---- AMBIENT LIGHT ----
    uint16_t lightRaw = myCodeCell.Light_AmbientRead();
    int lightValue = constrain((int)map(lightRaw, 0, lightMax, 0, 127), 0, 127);
    if (lightValue != lastLight) {
      sendUDP(tagLight, lightValue);
      lastLight = lightValue;
    }

    // ---- GRAVITY (magnitude, ~9.8 always, changes with orientation) ----
    float grx, gry, grz;
    myCodeCell.Motion_GravityRead(grx, gry, grz);
    float gravMag = sqrt((grx * grx) + (gry * gry) + (grz * grz));
    int gravValue = constrain((int)map((long)(gravMag * 10), 0, (long)(gravMax * 10), 0, 127), 0, 127);
    if (gravValue != lastGrav) {
      sendUDP(tagGrav, gravValue);
      lastGrav = gravValue;
    }

    // ---- LINEAR ACCELERATION (magnitude, "net" motion without gravity) ----
    float lx, ly, lz;
    myCodeCell.Motion_LinearAccRead(lx, ly, lz);
    float linMag = sqrt((lx * lx) + (ly * ly) + (lz * lz));
    int linValue = constrain((int)map((long)(linMag * 10), 0, (long)(linMax * 10), 0, 127), 0, 127);
    if (linValue != lastLin) {
      sendUDP(tagLin, linValue);
      lastLin = linValue;
    }

    // ---- IS THE PENDULUM STILL BEING USED? ----
    // Linear acceleration catches swinging, gyro catches spinning. Either one
    // resets the countdown.
    if (linMag > motionThresholdLin || gyroMag > motionThresholdGyro) {
      lastMotionTime = millis();
    }

    // ---- BATTERY (every ~5s) ----
    if (millis() - lastBatteryTime > batteryInterval) {
      lastBatLevel = myCodeCell.BatteryLevelRead();
      sendUDP(tagBat, lastBatLevel);
      Serial.print(DEVICE_NAME); Serial.print(" Bat: "); Serial.print(lastBatLevel); Serial.println("%");
      lastBatteryTime = millis();
    }

    // ---- AUTO-SLEEP ----
    // Skipped while plugged into USB (101 = charging, 102 = USB only), so the
    // board never sleeps on you while you are flashing or debugging it.
    if (idleTimeoutMinutes > 0 && lastBatLevel < 101) {
      if (millis() - lastMotionTime > (unsigned long)idleTimeoutMinutes * 60000UL) {
        Serial.println(">> No movement for the idle timeout.");
        goToSleep();
      }
    }
  }

  // Checked every pass, not only at 50 Hz, so a "sleep now" from Ableton lands
  // as fast as possible.
  handleIncomingUDP();
}