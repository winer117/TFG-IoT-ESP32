/*
 * SENSOR - ESP32 #3
 * Solo ESP-NOW: envía datos de temperatura y movimiento simulados
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "config.h"

struct DatosSensor {
  int idSensor;
  float temperatura;
  int movimiento;
  unsigned long tiempo;
  int msgID;
};

DatosSensor datos;
int msgCounter = 0;
bool esperandoACK = false;
unsigned long lastSend = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    esperandoACK = false;
    Serial.print("corecto");
  } else {
    Serial.print("incorrecto");
    esperandoACK = true;
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // ACK recibido
  esperandoACK = false;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, BRIDGE_MAC, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  datos.idSensor = 1;
  Serial.println("Sensor listo");
}

void loop() {
  if (millis() - lastSend > 5000 && !esperandoACK) {
    lastSend = millis();
    msgCounter++;
    datos.temperatura = 20.0 + random(0, 100)/10.0;
    datos.movimiento = random(0, 2);
    datos.tiempo = millis();
    datos.msgID = msgCounter;
    Serial.printf("📤 [#%d] Temp: %.1f°C | Mov: %s ", msgCounter, datos.temperatura, datos.movimiento ? "SI" : "NO");
    esp_now_send(BRIDGE_MAC, (uint8_t*)&datos, sizeof(datos));
  }
  delay(10);
}