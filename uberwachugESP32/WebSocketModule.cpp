#include "WebSocketModule.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "Hardware.h"
#include "Control.h"
#include "globals.h"

// WebSocket-Server auf Port 81
WebSocketsServer webSocket(81);


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
  doc["temperature"] = readTemperature();
  doc["current"] = readCurrent();         
  doc["vibration"] = readVibration();     
  doc["motor_status"] = motorStatus;
  doc["error_message"] = errorMessage;
  doc["speed"] = speed;
  doc["direction"] = direction_flag == 0 ? "Vorwärts" : "Rückwärts";

  // JSON-Dokument in String serialisieren
  String jsonString;
  serializeJson(doc, jsonString);

  // An alle verbundenen Clients senden
  webSocket.broadcastTXT(jsonString);
}

// Beispiel: Aktualisierung von Motorstatus und Fehler
void updateMotorStatus(String status, String error) {
  // Setze den Motorstatus nur, wenn kein Fehlerflag aktiv ist
  if (error_flag == 0) {
    motorStatus = status; // Normaler Status
    errorMessage = error; // Leere Fehlernachricht
  }

  // Falls Fehlerflag aktiv, Fehlerstatus beibehalten
  if (error_flag > 0) {
    motorStatus = "stopped";  // Motor bleibt gestoppt
    errorMessage = error;  // Fehlernachricht anzeigen
  }

  sendSensorAndStatusData(); // Daten an alle Clients senden
}
