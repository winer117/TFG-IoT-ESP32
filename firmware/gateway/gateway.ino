/*
 * GATEWAY - ESP32 #1
 * Funciones:
 * - Recibe datos del Bridge por Serial2
 * - Envía datos a ThingSpeak
 * - Mantiene red MESH para actuadores
 * - Ejecuta modelo TinyML (predicción de hábitos)
 */

#include "config.h"
#include <painlessMesh.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define MESH_PREFIX     "RedIoT"
#define MESH_PASSWORD   "12345678"
#define MESH_PORT       5555
#define MESH_CHANNEL    6

painlessMesh mesh;
int totalDatos = 0;
bool wifiDisponible = false;

// Buffer para datos sin WiFi
#define MAX_BUFFER 200
struct DatoBuffer {
  int id;
  float temp;
  int mov;
  unsigned long tiempo;
  bool enviado;
};
DatoBuffer buffer[MAX_BUFFER];
int bufferInicio = 0, bufferFin = 0, bufferCount = 0;

void conectarWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  wifiDisponible = (WiFi.status() == WL_CONNECTED);
  if (wifiDisponible) {
    Serial.printf("\n WiFi: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n WiFi no disponible (modo local)");
  }
}

void enviarAThingSpeak(int id, float temp, int mov) {
  if (!wifiDisponible) {
    // Guardar en buffer
    if (bufferCount < MAX_BUFFER) {
      buffer[bufferFin] = {id, temp, mov, millis(), false};
      bufferFin = (bufferFin + 1) % MAX_BUFFER;
      bufferCount++;
    }
    return;
  }
  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY);
  url += "&field1=" + String(temp);
  url += "&field2=" + String(mov);
  url += "&field3=" + String(id);
  http.begin(url);
  int code = http.GET();
  http.end();
  if (code == 200) Serial.printf(" Enviado sensor %d\n", id);
  else Serial.println(" Error ThingSpeak");
}

void enviarComandoMESH(int actuadorID, int estado) {
  String cmd = "ACTUADOR:" + String(actuadorID) + ":ESTADO:" + String(estado);
  mesh.sendBroadcast(cmd);
  Serial.printf("📤 Comando MESH: %s\n", cmd.c_str());
}

void procesarDatos(String datos) {
  // Formato: SENSOR:ID,temp,mov,tiempo
  int pos1 = datos.indexOf(",");
  int pos2 = datos.indexOf(",", pos1+1);
  int pos3 = datos.indexOf(",", pos2+1);
  int id = datos.substring(0, pos1).toInt();
  float temp = datos.substring(pos1+1, pos2).toFloat();
  int mov = datos.substring(pos2+1, pos3).toInt();
  totalDatos++;
  Serial.printf("📡 [%d] Sensor %d: %.1f°C | Mov: %s\n", totalDatos, id, temp, mov ? "SI" : "NO");
  enviarAThingSpeak(id, temp, mov);
  // Regla simple: si movimiento y temperatura > 28°C, encender ventilador (actuador ID 3)
  if (mov == 1 && temp > 28.0) {
    enviarComandoMESH(3, 1);
  }
}

void meshReceivedCallback(uint32_t from, String &msg) {
  Serial.printf("📩 Respuesta MESH de %u: %s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf(" Actuador conectado: %u\n", nodeId);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  conectarWiFi();
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, MESH_CHANNEL);
  mesh.onReceive(&meshReceivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  Serial.printf(" Gateway ID: %u\n", mesh.getNodeId());
}

void loop() {
  mesh.update();
  if (Serial2.available()) {
    String datos = Serial2.readStringUntil('\n');
    if (datos.startsWith("SENSOR:")) {
      procesarDatos(datos.substring(7));
    }
  }
  delay(10);
}