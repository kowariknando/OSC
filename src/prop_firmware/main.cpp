#include <Arduino.h>

// Single firmware for all sticks/diabolos.
// ONLY change wifi_configs.h (DEVICE_NAME) for each physical device.

#include <CodeCell.h>
#include "wifi_configs.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

CodeCell myCodeCell;
WiFiUDP Udp;

const char* ssid = WIFI_NAME;
const char* password = WIFI_PASS;
const char* pc_ip = WIFI_IP;
const int port = 9999;

// Tags built once, based on DEVICE_NAME
String tagInertia, tagBat, tagProx, tagState, tagAct, tagGyro, tagComp, tagRot, tagSteps;
String tagLight, tagGrav, tagLin;

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

void sendUDP(const String &tag, long value) {
  Udp.beginPacket(pc_ip, port);
  Udp.print(tag);
  Udp.print(" ");
  Udp.print(value);
  Udp.println(";");
  Udp.endPacket();
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

    // ---- BATTERY (every ~5s) ----
    if (millis() - lastBatteryTime > batteryInterval) {
      uint16_t lvl = myCodeCell.BatteryLevelRead();
      sendUDP(tagBat, lvl);
      Serial.print(DEVICE_NAME); Serial.print(" Bat: "); Serial.print(lvl); Serial.println("%");
      lastBatteryTime = millis();
    }
  }
}