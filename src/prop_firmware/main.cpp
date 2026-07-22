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

// --- PER-DEVICE SENSOR PROFILE (defined in wifi_configs.h, per physical prop) ---
// Only the sensors listed here are powered and fused on the board. Fewer sensors
// => less heat and less WiFi traffic. Each channel below is also gated on its
// flag, so disabled sensors are never read or sent.
// Example, for a prop that only drives the gyro plugin:
//   #define SENSOR_PROFILE (MOTION_GYRO)
// If wifi_configs.h does not define it, we fall back to the full set (the old
// behaviour), so existing devices keep working unchanged.
#ifndef SENSOR_PROFILE
#define SENSOR_PROFILE (LIGHT + MOTION_ACCELEROMETER + MOTION_STATE \
                        + MOTION_GYRO + MOTION_MAGNETOMETER + MOTION_ROTATION \
                        + MOTION_STEP_COUNTER + MOTION_ACTIVITY \
                        + MOTION_GRAVITY + MOTION_LINEAR_ACC)
#endif

// --- UDP PORT (per device, so independent single-variable plugins don't clash) ---
// Two Max `udpreceive` objects cannot share a port in one Live set, so each
// single-variable plugin listens on its own port. Set this per prop to match the
// plugin that should receive it (see max/single/README.md). Defaults to 9999
// (the old shared port) when not defined.
#ifndef UDP_PORT
#define UDP_PORT 9999
#endif

const char* ssid = WIFI_NAME;
const char* password = WIFI_PASS;
const char* pc_ip = WIFI_IP;
const int port = UDP_PORT;

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

// --- WAKE-ON-DOUBLE-TAP -----------------------------------------------------
// The IMU wakes the board on ANY single tap, so one stray knock (residual sway,
// the USB cable, someone brushing past) would reboot it and it looks like the
// board keeps restarting itself. To stop that, when the board wakes it does NOT
// come up straight away: it listens for you to keep tapping. Only if it counts
// wakeTapsNeeded taps within wakeListenMs does it stay awake and reconnect;
// otherwise it goes right back to sleep.
//
// So: to wake a board, TAP IT A FEW TIMES (a firm double/triple tap over about a
// second). One accidental tap is ignored. Because waking is a full reboot that
// takes a second or two, tapping several times is what reliably lands taps
// inside the listening window.
const unsigned long wakeListenMs = 4000;  // how long to listen for wake taps
const int wakeTapsNeeded = 2;             // taps within the window that confirm a wake

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
  // Only notify Max if we are actually online. When we bounce straight back to
  // sleep after a stray wake tap (see listenForWakeTaps), Wi-Fi isn't up yet.
  if (WiFi.status() == WL_CONNECTED) {
    sendUDP(tagSleep, 1);  // tell Max this device is going down on purpose
  }
  Serial.println(">> Going to sleep. Tap the pendulum a few times to wake it.");

  // Drain any pending IMU reports so the wake interrupt line (GPIO7) is idle
  // BEFORE we arm wake-on-LOW inside SleepTapTrigger(). If a report is still
  // pending the line is already low and the board wakes itself instantly - which
  // was a big part of the "it sleeps then restarts a second later" problem.
  for (int i = 0; i < 40; i++) {
    myCodeCell.Run(50);
    delay(3);
  }

  Serial.flush();
  delay(50);  // let the last packet leave before the radio dies

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  myCodeCell.SleepTapTrigger();  // never returns; wakes as a fresh boot
}

// Called once at boot, but only when we woke from deep sleep. The wake itself
// was one tap; here we require the user to keep tapping (wakeTapsNeeded taps
// within wakeListenMs) to prove the wake was deliberate. A single stray tap
// never reaches the count, so the board just goes back to sleep - it no longer
// reboots itself on every accidental knock. Returns true to stay awake.
bool listenForWakeTaps() {
  Serial.println(">> Woke up. Tap a few more times to keep it awake...");
  unsigned long start = millis();
  int taps = 0;
  while (millis() - start < wakeListenMs) {
    if (myCodeCell.Run(50)) {
      if (myCodeCell.Motion_TapDetectorRead()) {
        taps++;
        Serial.print(">> wake tap ");
        Serial.print(taps);
        Serial.print("/");
        Serial.println(wakeTapsNeeded);
        if (taps >= wakeTapsNeeded) {
          Serial.println(">> Confirmed - staying awake.");
          return true;
        }
      }
    }
  }
  Serial.println(">> Not enough taps - going back to sleep.");
  return false;
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

  // SENSOR_PROFILE picks which sensors stream; MOTION_TAP_DETECTOR is added so
  // the board can count wake taps (see listenForWakeTaps). It is not streamed as
  // a channel - it only feeds the wake logic.
  myCodeCell.Init(SENSOR_PROFILE | MOTION_TAP_DETECTOR);

  // If we woke from deep sleep, require a deliberate few taps before spending a
  // couple of seconds connecting to Wi-Fi. A single stray tap sends us straight
  // back to sleep here, so the board no longer reboots itself on its own. On a
  // normal power-up (flashing, plugging in the battery) WakeUpCheck() is false,
  // so this is skipped and the board boots straight through.
  if (myCodeCell.WakeUpCheck() && !listenForWakeTaps()) {
    goToSleep();  // never returns
  }

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

    // Set true by whichever motion sensor sees real movement this pass. Because
    // the sensor blocks below are gated on SENSOR_PROFILE, the motion check has
    // to live INSIDE each block (the magnitudes are scoped there) rather than
    // reading linMag/gyroMag afterwards. A prop with none of accelerometer /
    // gyro / linear enabled (e.g. a compass-only prop) has no motion signal, so
    // it will not auto-sleep on movement - set its timeout to 0 or sleep it from
    // Ableton instead.
    bool moving = false;

    // Each block is gated on its SENSOR_PROFILE flag. Because SENSOR_PROFILE is a
    // compile-time constant, the compiler folds these tests away and drops the
    // code for any sensor this device does not use — no read, no UDP send.

    // ---- INERTIA (accelerometer, smoothed magnitude) ----
    if (SENSOR_PROFILE & MOTION_ACCELEROMETER) {
      myCodeCell.Motion_AccelerometerRead(ax, ay, az);
      float current_magnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
      filtered_magnitude = (current_magnitude * smoothing_factor) + (filtered_magnitude * (1.0 - smoothing_factor));

      // Accelerometer magnitude sits at ~9.8 (gravity) when still; any real
      // movement pushes it away from that baseline.
      if (fabs(current_magnitude - min_reading) > motionThresholdLin) moving = true;

      int inertiaValue = map((long)(filtered_magnitude * 10), (long)(min_reading * 10), (long)(max_reading * 10), 0, 127);
      inertiaValue = constrain(inertiaValue, 0, 127);
      if (inertiaValue != lastInertia) {
        sendUDP(tagInertia, inertiaValue);
        lastInertia = inertiaValue;
      }
    }

    // ---- PROXIMITY ----
    if (SENSOR_PROFILE & LIGHT) {
      uint16_t proxRaw = myCodeCell.Light_ProximityRead();
      int proxValue = constrain((int)map(proxRaw, 0, proxMax, 0, 127), 0, 127);
      if (proxValue != lastProx) {
        sendUDP(tagProx, proxValue);
        lastProx = proxValue;
      }
    }

    // ---- STATE: 1=Table, 2=Stationary, 3=Stable, 4=Motion ----
    if (SENSOR_PROFILE & MOTION_STATE) {
      int state = myCodeCell.Motion_StateRead();
      if (state != lastState) {
        sendUDP(tagState, state);
        lastState = state;
      }
    }

    // ---- ACTIVITY ----
    if (SENSOR_PROFILE & MOTION_ACTIVITY) {
      int act = myCodeCell.Motion_ActivityRead();
      if (act != lastAct) {
        sendUDP(tagAct, act);
        lastAct = act;
      }
    }

    // ---- GYROSCOPE (magnitude) ----
    if (SENSOR_PROFILE & MOTION_GYRO) {
      float gx, gy, gz;
      myCodeCell.Motion_GyroRead(gx, gy, gz);
      float gyroMag = sqrt((gx * gx) + (gy * gy) + (gz * gz));
      if (gyroMag > motionThresholdGyro) moving = true;
      int gyroValue = constrain((int)map((long)gyroMag, 0, (long)gyroMax, 0, 127), 0, 127);
      if (gyroValue != lastGyro) {
        sendUDP(tagGyro, gyroValue);
        lastGyro = gyroValue;
      }
    }

    // ---- MAGNETOMETER (magnitude, used as "compass") ----
    if (SENSOR_PROFILE & MOTION_MAGNETOMETER) {
      float mx, my, mz;
      myCodeCell.Motion_MagnetometerRead(mx, my, mz);
      float compMag = sqrt((mx * mx) + (my * my) + (mz * mz));
      int compValue = constrain((int)map((long)compMag, 0, (long)compMax, 0, 127), 0, 127);
      if (compValue != lastComp) {
        sendUDP(tagComp, compValue);
        lastComp = compValue;
      }
    }

    // ---- ROTATION (yaw, -180..180 -> 0..127) ----
    if (SENSOR_PROFILE & MOTION_ROTATION) {
      float roll, pitch, yaw;
      myCodeCell.Motion_RotationRead(roll, pitch, yaw);
      int rotValue = constrain((int)map((long)yaw, -180, 180, 0, 127), 0, 127);
      if (rotValue != lastRot) {
        sendUDP(tagRot, rotValue);
        lastRot = rotValue;
      }
    }

    // ---- STEPS ----
    if (SENSOR_PROFILE & MOTION_STEP_COUNTER) {
      uint16_t steps = myCodeCell.Motion_StepCounterRead();
      if (steps != lastSteps) {
        sendUDP(tagSteps, (long)steps);
        lastSteps = steps;
      }
    }

    // ---- AMBIENT LIGHT ----
    if (SENSOR_PROFILE & LIGHT) {
      uint16_t lightRaw = myCodeCell.Light_AmbientRead();
      int lightValue = constrain((int)map(lightRaw, 0, lightMax, 0, 127), 0, 127);
      if (lightValue != lastLight) {
        sendUDP(tagLight, lightValue);
        lastLight = lightValue;
      }
    }

    // ---- GRAVITY (magnitude, ~9.8 always, changes with orientation) ----
    if (SENSOR_PROFILE & MOTION_GRAVITY) {
      float grx, gry, grz;
      myCodeCell.Motion_GravityRead(grx, gry, grz);
      float gravMag = sqrt((grx * grx) + (gry * gry) + (grz * grz));
      int gravValue = constrain((int)map((long)(gravMag * 10), 0, (long)(gravMax * 10), 0, 127), 0, 127);
      if (gravValue != lastGrav) {
        sendUDP(tagGrav, gravValue);
        lastGrav = gravValue;
      }
    }

    // ---- LINEAR ACCELERATION (magnitude, "net" motion without gravity) ----
    if (SENSOR_PROFILE & MOTION_LINEAR_ACC) {
      float lx, ly, lz;
      myCodeCell.Motion_LinearAccRead(lx, ly, lz);
      float linMag = sqrt((lx * lx) + (ly * ly) + (lz * lz));
      if (linMag > motionThresholdLin) moving = true;
      int linValue = constrain((int)map((long)(linMag * 10), 0, (long)(linMax * 10), 0, 127), 0, 127);
      if (linValue != lastLin) {
        sendUDP(tagLin, linValue);
        lastLin = linValue;
      }
    }

    // ---- IS THE PENDULUM STILL BEING USED? ----
    // Any enabled motion sensor that saw movement this pass resets the countdown.
    if (moving) {
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