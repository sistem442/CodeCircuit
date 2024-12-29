#include "WebSocketModule.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "Hardware.h"
#include "Control.h"

// WebSocket-Server auf Port 81
WebSocketsServer webSocket(81);

// Globale Variablen für Motorstatus und Fehlerbeschreibung
String motorStatus = "running"; // Alternativ: "stopped"
String errorMessage = "";       // Fehlerbeschreibung

// Prototypen
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendSensorAndStatusData();

void setupWebSocket() {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent); // Event-Handler registrieren
  Serial.println("WebSocket-Server gestartet");
}

void handleWebSocket() {
  webSocket.loop();
}

// WebSocket-Event-Handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Client verbunden\n", num);
      sendSensorAndStatusData(); // Sende Initialdaten an den neuen Client
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client getrennt\n", num);
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Nachricht empfangen: %s\n", num, payload);
      // Optional: Verarbeitung von Nachrichten vom Client
      break;
    default:
      break;
  }
}

// Funktion zum Senden von Sensordaten und Motorstatus
void sendSensorAndStatusData() {
  StaticJsonDocument<300> doc;

  // Sensordaten hinzufügen
  doc["temperature"] = readTemperature(); // Sensor-Funktion implementieren
  doc["current"] = readCurrent();         // Sensor-Funktion implementieren
  doc["vibration"] = readVibration();     // Sensor-Funktion implementieren

  // Motorstatus hinzufügen
  doc["motor_status"] = motorStatus;
  doc["error_message"] = errorMessage;

  // JSON-Dokument in String serialisieren
  String jsonString;
  serializeJson(doc, jsonString);

  // An alle verbundenen Clients senden
  webSocket.broadcastTXT(jsonString);
}

// Beispiel: Aktualisierung von Motorstatus und Fehler
void updateMotorStatus(String status, String error) {
  motorStatus = status;
  errorMessage = error;
  sendSensorAndStatusData(); // Aktualisierte Daten an alle Clients senden
}
