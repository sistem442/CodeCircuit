#include "Hardware.h"
#include "Control.h"
#include "WebServerModule.h"
#include "Utilities.h"
#include "WebSocketModule.h"

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("ESP32 initialisation!");
  setupHardware();
  setupWebServer();
  setupWebSocket();
  setupControl();
  Serial.println("ESP32 bereit!");
}

void loop() {
  handleWebServer();
  handleWebSocket();
  updateControl();
  handleRoot();
  
  // Sensordaten senden
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 2000) {
    sendSensorAndStatusData();
    lastSendTime = millis();
  }
}
/*Ausgabe der Temperatur
  Serial.print("Temperatur: ");
  Serial.print(tempC);
  Serial.println(" °C");
*/
/* if (millis() - speedPrintTime >= 5000) {
    speedPrintTime = millis();  // Zeitpunkt aktualisieren
    // Serial.print("Speed: ");
    // Serial.println(speed);

    // Serial.print("Current:       ");
    // Serial.print(current_mA);
    // Serial.println(" mA");

    void handlePOT() {
  String potValue = String(analogRead(32));
  Serial.print("Potentiometer: ");
  Serial.println(potValue);
  server.send(200, "text/plane", potValue);
}
  }*/
