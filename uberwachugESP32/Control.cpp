#include "Control.h"
#include <Arduino.h>
#include "globals.h"
#include "Utilities.h"
#include "PinDefinitions.h"
#include "Hardware.h"
#include "WebSocketModule.h"

unsigned long lastDebounceTimeStartStop = 0;
unsigned long lastDebounceTimeConfirm = 0;
unsigned long lastDebounceTimeDirection = 0;

void setupControl() {
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIRETION_A_PIN, OUTPUT);
  pinMode(MOTOR_DIRETION_B_PIN, OUTPUT);
  pinMode(START_STOP_BUTTON_PIN, INPUT);
  pinMode(MOTOR_DIRECTION_BUTTON_PIN, INPUT);
  pinMode(CONNFIRMATION_BUTTON_PIN, INPUT);
  pinMode(START_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, LOW);
}

void updateControl() {

  potValue = analogRead(MOTOR_SPEED_PIN);
  speed = map(potValue, 0, 4095, 0, 255);  // Anpassung für 12-Bit ADC des ESP32
  ledcWrite(0, speed);
  updateMotorStatus(motorStatus, errorMessage);

   // Überwachung von Temperatur, Strom und Vibrationen
  float currentTemp = readTemperature();
  float current_mA = readCurrent();
  int vibration = readVibration();

  // Motor Start/Stop
  if (debounce(START_STOP_BUTTON_PIN, lastDebounceTimeStartStop)) {
    if (on_flag == 0) {
      digitalWrite(START_LED_PIN, HIGH);
      digitalWrite(MOTOR_ENABLE_PIN, HIGH);
      ledcWrite(0, speed);
      digitalWrite(MOTOR_DIRETION_A_PIN, HIGH);
      digitalWrite(MOTOR_DIRETION_B_PIN, LOW);
      on_flag = 1;
      Serial.println("Motor turned on");
      motorStatus = "Running";
    } else {
      ledcWrite(0, 0);
      digitalWrite(MOTOR_ENABLE_PIN, LOW);
      digitalWrite(START_LED_PIN, LOW);
      on_flag = 0;
      Serial.println("Motor turned off");
      motorStatus = "Stopped, manually turned off";
    }
  }

  // Direction Control
  if (debounce(MOTOR_DIRECTION_BUTTON_PIN, lastDebounceTimeDirection)) {
    if (direction_flag == 0) {
      digitalWrite(MOTOR_DIRETION_A_PIN, LOW);
      digitalWrite(MOTOR_DIRETION_B_PIN, HIGH);
      direction_flag = 1;
      Serial.println("Direction changed");
      motorStatus = "Running, Direction changed";
    } else {
      digitalWrite(MOTOR_DIRETION_A_PIN, HIGH);
      digitalWrite(MOTOR_DIRETION_B_PIN, LOW);
      direction_flag = 0;
      Serial.println("Direction reset");
      motorStatus = "Running, Direction reset";
    }
  }

  // Fehler bestätigen
  if (debounce(CONNFIRMATION_BUTTON_PIN, lastDebounceTimeConfirm)) {
    error_flag = 0;
    digitalWrite(ERROR_LED_PIN, LOW);
    Serial.println("Error cleared");
    motorStatus = "Turned off.";
    errorMessage = "No errors.";
    on_flag = 0;
  }

  if (currentTemp > 20) {  // Beispielgrenzwert für Temperatur
    error_flag = 2;
    Serial.println("Error: Temperature too high! (" + String(currentTemp) + " °C)");
    motorStatus = "Stopped!";
    errorMessage = "Temperature too high!";
  }

  if (current_mA > 200) {  // Beispielgrenzwert für Strom
    error_flag = 3;
    Serial.println("Error: Current too high! (" + String(current_mA) + " mA)");
    motorStatus = "Stopped!";
    errorMessage = "Current too high!";
  }

  if (vibration == HIGH) {  // Vibration erkannt
    error_flag = 4;
    Serial.println("Error: Vibration detected!");
    motorStatus = "Stopped!";
    errorMessage = "Vibration detected!";
  }

  // Fehlerlogik
  if (error_flag > 0) {
    ledcWrite(0, 0);  // Motor ausschalten
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, HIGH);
    digitalWrite(START_LED_PIN, LOW);
  }
}
