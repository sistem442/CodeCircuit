#include "WebServerModule.h"
#include <WiFi.h>
#include <WebServer.h>
#include "WebSocketModule.h"
#include "globals.h"
#include "webpage.h"

WebServer server(80);

const char* ssid = "MagentaWLAN-IRVV";
const char* password = "77305313069659970872";

void setupWebServer() {
    // Wi-Fi setup
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.println(WiFi.localIP());

  // Define routes
  server.on("/", handleRoot);
  server.on("/readPOT", handlePOT);
  server.on("/daten", handleData);

  // Start server
  server.begin();
  Serial.println("Server started");
}

void handleWebServer() {
    server.handleClient();
}
void handleRoot() {
  server.send(200, "text/html", webpageCode);
  Serial.println("handleRoot");
}

void handleData() {
  String json = "{";
  json += "\"temperature\": " + String(tempC, 1) + ",";
  json += "\"current\": " + String(current_mA, 2) + ",";
  json += "\"vibration\": " + String(vibration == 0 ? 1 : 0);
  json += "}";

  server.send(200, "application/json", json);
}

 void handlePOT() {
  String potValue = String(analogRead(32));
  Serial.print("Potentiometer: ");
  Serial.println(potValue);
  server.send(200, "text/plane", potValue);
}



