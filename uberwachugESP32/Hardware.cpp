#include "Hardware.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Arduino.h>
#include "globals.h"
#include "PinDefinitions.h"

Adafruit_INA219 ina219;

void setupHardware() {
  Wire.begin(21, 22);  // SDA, SCL für INA sensor

  // INA219 initialisieren
  if (!ina219.begin()) {
    Serial.println("INA219 nicht gefunden. Überprüfe die Verkabelung!");
    while (1) {
      delay(10);
    }
  }
  pinMode(VIBRATION_SENSOR_PIN, INPUT);    // Vibration sensor
  pinMode(START_LED_PIN, OUTPUT);  // Running light
  pinMode(ERROR_LED_PIN, OUTPUT);  // Error light
}
 
float readTemperature() {
  int tempReading = analogRead(32);
  double tempK = log(10000.0 * ((4095.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK)) * tempK);
  tempC = tempK - 273.15;
  tempC = tempC + 4; //Offset
  if (tempC > 30) {
    error_flag = 2;
  }
  return tempC;
}

float readCurrent() {
  current_mA = ina219.getCurrent_mA();
  return ina219.getCurrent_mA();
}

int readVibration() {
  return digitalRead(5);
}
