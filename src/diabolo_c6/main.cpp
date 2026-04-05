#include <Arduino.h>

// Project: CodeCell C6 Pendulum (Improved Sensitivity & Battery Monitor)
// Sends data to PureData for Korg Minilogue XD Cutoff control

#include <CodeCell.h>
#include "wifi_configs.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

CodeCell myCodeCell;
WiFiUDP Udp;

// --- TUS DATOS ---
const char* ssid = WIFI_NAME;      
const char* password = WIFI_PASS;    
const char* ip_pc = WIFI_IP;          
const int puerto = 9999;                     
// -----------------

float x, y, z;
float magnitud_filtrada = 0;

// --- CONFIGURACIÓN DE SENSIBILIDAD ---
float factor_suavizado = 0.05; //0.05 para los diabolos // ahora palos 0.18
float lectura_minima = 9.8;  // Gravedad en reposo
// PENDULO: 11.5 | DIABOLO: 20.0 (Bájalo más si quieres que sea aún más sensible)
float lectura_maxima = 20.0; 
int lastMidiValue = -1;

// --- CONFIGURACIÓN DE REPOSO (SLEEP) ---
unsigned long lastMovementTime = 0;
const unsigned long timeToSleep = 600000; 
const float movementThreshold = 1.5;     

// --- TEMPORIZADOR BATERÍA (NUEVO) ---
unsigned long lastBatteryTime = 0;
const unsigned long batteryInterval = 5000; 

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(MOTION_ACCELEROMETER + MOTION_TAP_DETECTOR);

  // Anti-bloqueo WiFi
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
  
  Udp.begin(puerto);
  
  magnitud_filtrada = lectura_minima;
  lastMovementTime = millis();
}

void loop() {
  if (myCodeCell.Run(50)) { // 50Hz
    myCodeCell.Motion_AccelerometerRead(x, y, z);

    // 1. Calcular Magnitud
    float magnitud_actual = sqrt((x * x) + (y * y) + (z * z));

    // 2. Filtro suavizado
    magnitud_filtrada = (magnitud_actual * factor_suavizado) + (magnitud_filtrada * (1.0 - factor_suavizado));

    // --- 3. LÓGICA DE REPOSO ---
    if (abs(magnitud_filtrada - 9.8) > movementThreshold) {
      lastMovementTime = millis();
    }

    //if (millis() - lastMovementTime > timeToSleep) {
    //  Serial.println("Péndulo inactivo. Entrando en Deep Sleep...");
    //  delay(100); 
    //  WiFi.disconnect(); 
    //  myCodeCell.SleepTapTrigger();
    //}

    // --- 4. MAPEO Y ENVÍO UDP (MOVIMIENTO) ---
    // Mapeo con los nuevos rangos de sensibilidad
    int midiValue = map((long)(magnitud_filtrada * 10), (long)(lectura_minima * 10), (long)(lectura_maxima * 10), 0, 127);
    midiValue = constrain(midiValue, 0, 127);

    if (midiValue != lastMidiValue) {
      Udp.beginPacket(ip_pc, puerto);
      Udp.print("sensor_c6_4 "); 
      Udp.print(midiValue);
      Udp.println(";");
      Udp.endPacket();
      lastMidiValue = midiValue;
    }

    // --- 5. ENVÍO UDP (BATERÍA INTEGRADO) ---
    if (millis() - lastBatteryTime > batteryInterval) {
      uint16_t lvl = myCodeCell.BatteryLevelRead();
      
      Udp.beginPacket(ip_pc, puerto);
      Udp.print("bateria_c6_4 "); // Nota: He puesto c6_ para que coincida con el ID del sensor
      Udp.print(lvl);
      Udp.println(";");
      Udp.endPacket();
      
      // Monitorización por Serial para tu control
      Serial.print("Bat: "); Serial.print(lvl); Serial.println("%");
      
      lastBatteryTime = millis();
    }
  }
}