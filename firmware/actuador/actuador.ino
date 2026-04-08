/*
 * ACTUADOR - ESP32 #4
 * Solo MESH: recibe comandos y controla un LED/relé
 */

#include <painlessMesh.h>
#include "config.h"

#define MESH_PREFIX     "RedIoT"
#define MESH_PASSWORD   "12345678"
#define MESH_PORT       5555
#define PIN_ACTUADOR    2

painlessMesh mesh;
int comandosRecibidos = 0;
int estadoActual = 0;

void controlar(int estado) {
  estadoActual = estado;
  digitalWrite(PIN_ACTUADOR, estado);
  Serial.println(estado ? "💡 ENCENDIDO" : "💡 APAGADO");
}

void receivedCallback(uint32_t from, String &msg) {
  comandosRecibidos++;
  Serial.printf(" Comando #%d de %u: %s\n", comandosRecibidos, from, msg.c_str());
  if (msg.startsWith("ACTUADOR:")) {
    int id = msg.substring(9, msg.indexOf(":", 9)).toInt();
    int estado = msg.substring(msg.lastIndexOf(":")+1).toInt();
    if (id == ID_ACTUADOR) {
      controlar(estado);
      mesh.sendSingle(from, "OK");
    }
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf(" Conectado a Gateway: %u\n", nodeId);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ACTUADOR, OUTPUT);
  digitalWrite(PIN_ACTUADOR, LOW);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_STA, 6);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  Serial.printf("Actuador ID %d listo\n", ID_ACTUADOR);
}

void loop() {
  mesh.update();
  delay(10);
}