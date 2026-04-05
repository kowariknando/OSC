#include <Arduino.h>


// Project: CodeCell C3 VR Controller with RESET/ZERO function & IP Sender
// Sends Axis-Angle data with custom Zero-Position offset

#include <CodeCell.h>
#include "wifi_configs.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

CodeCell myCodeCell;
WiFiUDP Udp;

// --- TUS DATOS DE RED ---    
const char* ssid = WIFI_NAME;      
const char* password = WIFI_PASS;    
const char* ip_pc = WIFI_IP;   
const int puerto = 9999;                     
// ------------------------

// Variables para la fusión de sensores
float roll, pitch, yaw;

// Cuaterniones para el cálculo del offset (Reset)
float q_offset_w = 1.0, q_offset_x = 0.0, q_offset_y = 0.0, q_offset_z = 0.0;

// Variables globales para el cálculo continuo
float qw_now = 1.0, qx_now = 0.0, qy_now = 0.0, qz_now = 0.0;

// Temporizador para enviar la IP repetidamente
unsigned long lastIpTime = 0;

// Buffer para recibir mensajes de PureData
char packetBuffer[255]; 

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(MOTION_ROTATION);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Le damos 2 segundos de respiro para que te dé tiempo a abrir el Monitor Serie
  delay(2000); 
  
  Serial.println("\nWiFi Conectado!");
  Serial.print("La IP del CodeCell es: ");
  Serial.println(WiFi.localIP());
  
  Udp.begin(puerto);
}

void loop() {
  // --- 1. ENVIAR IP A PURE DATA CADA 5 SEGUNDOS ---
  if (millis() - lastIpTime > 5000) {
    Udp.beginPacket(ip_pc, puerto);
    Udp.print("ip_info ");
    Udp.print(WiFi.localIP());
    Udp.println(";");
    Udp.endPacket();
    
    // También lo imprimimos por Serial por si acaso
    Serial.print("Mi IP es: "); Serial.println(WiFi.localIP());
    
    lastIpTime = millis();
  }

  // --- 2. ESCUCHAR SI PURE DATA MANDA UN "RESET" ---
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String msg = String(packetBuffer);
    if (msg.indexOf("reset") >= 0) {
      // Capturamos la orientación actual para que sea el nuevo "Cero"
      q_offset_w = qw_now; 
      q_offset_x = -qx_now; // Conjugado = negar ejes X, Y, Z
      q_offset_y = -qy_now;
      q_offset_z = -qz_now;
      Serial.println("¡Posición Cero reseteada!");
    }
  }

  // --- 3. LEER SENSOR Y CALCULAR ORIENTACIÓN RELATIVA ---
  if (myCodeCell.Run(50)) { 
    myCodeCell.Motion_RotationRead(roll, pitch, yaw);

    float r = roll * PI / 180.0;
    float p = pitch * PI / 180.0;
    float y = yaw * PI / 180.0;

    float cy = cos(y * 0.5); float sy = sin(y * 0.5);
    float cp = cos(p * 0.5); float sp = sin(p * 0.5);
    float cr = cos(r * 0.5); float sr = sin(r * 0.5);

    float qw_raw = cr * cp * cy + sr * sp * sy;
    float qx_raw = sr * cp * cy - cr * sp * sy;
    float qy_raw = cr * sp * cy + sr * cp * sy;
    float qz_raw = cr * cp * sy - sr * sp * cy;

    // APLICAR EL OFFSET (Multiplicación de Cuaterniones)
    float qw = qw_raw * q_offset_w - qx_raw * q_offset_x - qy_raw * q_offset_y - qz_raw * q_offset_z;
    float qx = qw_raw * q_offset_x + qx_raw * q_offset_w + qy_raw * q_offset_z - qz_raw * q_offset_y;
    float qy = qw_raw * q_offset_y - qx_raw * q_offset_z + qy_raw * q_offset_w + qz_raw * q_offset_x;
    float qz = qw_raw * q_offset_z + qx_raw * q_offset_y - qy_raw * q_offset_x + qz_raw * q_offset_w;

    // Guardamos los valores actuales para el próximo reset
    qw_now = qw_raw; qx_now = qx_raw; qy_now = qy_raw; qz_now = qz_raw;

    // Convertir Cuaternión relativo a Eje-Ángulo para GEM
    float angle_rad = 2.0 * acos(qw);
    float angle_deg = angle_rad * 180.0 / PI;
    float den = sqrt(1.0 - qw * qw);
    float ax, ay, az;
    if (den < 0.0001) { ax = 1.0; ay = 0.0; az = 0.0; } 
    else { ax = qx / den; ay = qy / den; az = qz / den; }

    // Enviar datos a PureData
    Udp.beginPacket(ip_pc, puerto);
    Udp.print("axis_c3_3 "); 
    Udp.print(angle_deg); Udp.print(" ");
    Udp.print(ax); Udp.print(" ");
    Udp.print(ay); Udp.print(" ");
    Udp.print(az);
    Udp.println(";"); 
    Udp.endPacket();
  }
}