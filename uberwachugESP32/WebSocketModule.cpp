#include "WebSocketModule.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "Hardware.h"

// WebSocket-Server auf Port 81
WebSocketsServer webSocket(81);

// Prototyp der Event-Handler-Funktion
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setupWebSocket() {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent); // Event-Handler registrieren
  Serial.println("WebSocket-Server gestartet");
}

void handleWebSocket() {
  webSocket.loop();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Client verbunden\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client getrennt\n", num);
      break;
    default:
      break;
  }
}

void sendSensorData() {
  StaticJsonDocument<200> doc;
  doc["temperature"] = readTemperature();
  doc["current"] = readCurrent();
  doc["vibration"] = readVibration();

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
}
