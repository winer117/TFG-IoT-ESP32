/*
 * BRIDGE - ESP32 #2
 * Solo ESP-NOW: recibe datos del sensor y los reenvía por Serial2 al Gateway
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

int totalMensajes = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  DatosSensor datos;
  memcpy(&datos, incomingData, sizeof(datos));
  totalMensajes++;
  // Enviar ACK
  int ack = datos.msgID;
  esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
  // Enviar al Gateway por Serial2
  Serial2.printf("SENSOR:%d,%.1f,%d,%lu\n", datos.idSensor, datos.temperatura, datos.movimiento, datos.tiempo);
  Serial.printf("📡 [%d] Sensor %d: %.1f°C | Mov:%s\n", totalMensajes, datos.idSensor, datos.temperatura, datos.movimiento ? "SI" : "NO");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  // Registrar sensor como peer (su MAC está en config.h)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, SENSOR_MAC, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  Serial.println("Bridge listo");
}

void loop() {
  delay(10);
}