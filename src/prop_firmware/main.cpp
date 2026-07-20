#include <Arduino.h>

// Firmware unic per a tots els sticks/diabolos.
// NOMES canvia wifi_configs.h (DEVICE_NAME) per cada dispositiu fisic.

#include <CodeCell.h>
#include "wifi_configs.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

CodeCell myCodeCell;
WiFiUDP Udp;

const char* ssid = WIFI_NAME;
const char* password = WIFI_PASS;
const char* ip_pc = WIFI_IP;
const int puerto = 9999;

// Etiquetes construides una sola vegada, a partir de DEVICE_NAME
String tagInercia, tagBat, tagProx, tagState, tagAct, tagGyro, tagComp, tagRot, tagSteps;
String tagLlum, tagGrav, tagLin;

float ax, ay, az;
float magnitud_filtrada = 0;

// --- SENSIBILITAT INERCIA (ajusta per stick vs diabolo si cal) ---
float factor_suavizado = 0.05;
float lectura_minima = 9.8;
float lectura_maxima = 20.0;
int lastInercia = -1;

// --- PROXIMITAT ---
const uint16_t proxMaxima = 1000;
int lastProx = -1;

// --- ESTAT (On table / Stationary / Stable / Motion) ---
int lastState = -1;

// --- ACTIVITAT (Driving/Cycling/Walking/Still/Tilting/Running/Climbing) ---
int lastAct = -1;

// --- GIROSCOPI (magnitud) ---
const float gyroMaxima = 250.0;
int lastGyro = -1;

// --- MAGNETOMETRE (magnitud, com a "compass") ---
const float compMaxima = 100.0;
int lastComp = -1;

// --- ROTACIO (yaw) ---
int lastRot = -1;

// --- PASSOS ---
uint16_t lastSteps = 0;

// --- LLUM AMBIENT ---
const uint16_t llumMaxima = 1000;
int lastLlum = -1;

// --- GRAVETAT (magnitud, hauria de ser ~9.8 sempre, canvia amb orientacio) ---
const float gravMaxima = 12.0;
int lastGrav = -1;

// --- ACCELERACIO LINEAL (magnitud, "moviment net" sense la gravetat) ---
const float linMaxima = 15.0;
int lastLin = -1;

// --- BATERIA ---
unsigned long lastBatteryTime = 0;
const unsigned long batteryInterval = 5000;

void sendUDP(const String &tag, long value) {
  Udp.beginPacket(ip_pc, puerto);
  Udp.print(tag);
  Udp.print(" ");
  Udp.print(value);
  Udp.println(";");
  Udp.endPacket();
}

void setup() {
  Serial.begin(115200);

  tagInercia = String(DEVICE_NAME) + "_inercia";
  tagBat     = String(DEVICE_NAME) + "_bat";
  tagProx    = String(DEVICE_NAME) + "_prox";
  tagState   = String(DEVICE_NAME) + "_state";
  tagAct     = String(DEVICE_NAME) + "_act";
  tagGyro    = String(DEVICE_NAME) + "_gyro";
  tagComp    = String(DEVICE_NAME) + "_comp";
  tagRot     = String(DEVICE_NAME) + "_rot";
  tagSteps   = String(DEVICE_NAME) + "_steps";
  tagLlum    = String(DEVICE_NAME) + "_llum";
  tagGrav    = String(DEVICE_NAME) + "_grav";
  tagLin     = String(DEVICE_NAME) + "_lin";

  myCodeCell.Init(LIGHT + MOTION_ACCELEROMETER + MOTION_STATE
                  + MOTION_GYRO + MOTION_MAGNETOMETER + MOTION_ROTATION
                  + MOTION_STEP_COUNTER + MOTION_ACTIVITY
                  + MOTION_GRAVITY + MOTION_LINEAR_ACC);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.print("Conectando a WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Conectado!");
  Serial.print("Device: "); Serial.println(DEVICE_NAME);

  Udp.begin(puerto);
  magnitud_filtrada = lectura_minima;
}

void loop() {
  if (myCodeCell.Run(50)) {

    // ---- INERCIA (acceleròmetre, magnitud suavitzada) ----
    myCodeCell.Motion_AccelerometerRead(ax, ay, az);
    float magnitud_actual = sqrt((ax * ax) + (ay * ay) + (az * az));
    magnitud_filtrada = (magnitud_actual * factor_suavizado) + (magnitud_filtrada * (1.0 - factor_suavizado));

    int inerciaValue = map((long)(magnitud_filtrada * 10), (long)(lectura_minima * 10), (long)(lectura_maxima * 10), 0, 127);
    inerciaValue = constrain(inerciaValue, 0, 127);
    if (inerciaValue != lastInercia) {
      sendUDP(tagInercia, inerciaValue);
      lastInercia = inerciaValue;
    }

    // ---- PROXIMITAT ----
    uint16_t proxRaw = myCodeCell.Light_ProximityRead();
    int proxValue = constrain((int)map(proxRaw, 0, proxMaxima, 0, 127), 0, 127);
    if (proxValue != lastProx) {
      sendUDP(tagProx, proxValue);
      lastProx = proxValue;
    }

    // ---- ESTAT: 1=Taula, 2=Estacionari, 3=Estable, 4=Movent ----
    int state = myCodeCell.Motion_StateRead();
    if (state != lastState) {
      sendUDP(tagState, state);
      lastState = state;
    }

    // ---- ACTIVITAT ----
    int act = myCodeCell.Motion_ActivityRead();
    if (act != lastAct) {
      sendUDP(tagAct, act);
      lastAct = act;
    }

    // ---- GIROSCOPI (magnitud) ----
    float gx, gy, gz;
    myCodeCell.Motion_GyroRead(gx, gy, gz);
    float gyroMag = sqrt((gx * gx) + (gy * gy) + (gz * gz));
    int gyroValue = constrain((int)map((long)gyroMag, 0, (long)gyroMaxima, 0, 127), 0, 127);
    if (gyroValue != lastGyro) {
      sendUDP(tagGyro, gyroValue);
      lastGyro = gyroValue;
    }

    // ---- MAGNETOMETRE (magnitud, com a "compass") ----
    float mx, my, mz;
    myCodeCell.Motion_MagnetometerRead(mx, my, mz);
    float compMag = sqrt((mx * mx) + (my * my) + (mz * mz));
    int compValue = constrain((int)map((long)compMag, 0, (long)compMaxima, 0, 127), 0, 127);
    if (compValue != lastComp) {
      sendUDP(tagComp, compValue);
      lastComp = compValue;
    }

    // ---- ROTACIO (yaw, -180..180 -> 0..127) ----
    float roll, pitch, yaw;
    myCodeCell.Motion_RotationRead(roll, pitch, yaw);
    int rotValue = constrain((int)map((long)yaw, -180, 180, 0, 127), 0, 127);
    if (rotValue != lastRot) {
      sendUDP(tagRot, rotValue);
      lastRot = rotValue;
    }

    // ---- PASSOS ----
    uint16_t steps = myCodeCell.Motion_StepCounterRead();
    if (steps != lastSteps) {
      sendUDP(tagSteps, (long)steps);
      lastSteps = steps;
    }

    // ---- LLUM AMBIENT ----
    uint16_t llumRaw = myCodeCell.Light_AmbientRead();
    int llumValue = constrain((int)map(llumRaw, 0, llumMaxima, 0, 127), 0, 127);
    if (llumValue != lastLlum) {
      sendUDP(tagLlum, llumValue);
      lastLlum = llumValue;
    }

    // ---- GRAVETAT (magnitud, ~9.8 sempre, canvia amb l'orientacio) ----
    float grx, gry, grz;
    myCodeCell.Motion_GravityRead(grx, gry, grz);
    float gravMag = sqrt((grx * grx) + (gry * gry) + (grz * grz));
    int gravValue = constrain((int)map((long)(gravMag * 10), 0, (long)(gravMaxima * 10), 0, 127), 0, 127);
    if (gravValue != lastGrav) {
      sendUDP(tagGrav, gravValue);
      lastGrav = gravValue;
    }

    // ---- ACCELERACIO LINEAL (magnitud, moviment "net" sense gravetat) ----
    float lx, ly, lz;
    myCodeCell.Motion_LinearAccRead(lx, ly, lz);
    float linMag = sqrt((lx * lx) + (ly * ly) + (lz * lz));
    int linValue = constrain((int)map((long)(linMag * 10), 0, (long)(linMaxima * 10), 0, 127), 0, 127);
    if (linValue != lastLin) {
      sendUDP(tagLin, linValue);
      lastLin = linValue;
    }

    // ---- BATERIA (cada ~5s) ----
    if (millis() - lastBatteryTime > batteryInterval) {
      uint16_t lvl = myCodeCell.BatteryLevelRead();
      sendUDP(tagBat, lvl);
      Serial.print(DEVICE_NAME); Serial.print(" Bat: "); Serial.print(lvl); Serial.println("%");
      lastBatteryTime = millis();
    }
  }
}
