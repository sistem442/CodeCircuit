// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>

// Deklaration der globalen Variablen
extern int error_flag;
extern int potValue;  // Variable zum Speichern des Potentiometerwerts
extern int on_flag;
extern int direction_flag;
extern int speed;
extern double tempC;
extern double current_mA; 
extern int vibration;
extern String motorStatus;
extern String errorMessage;
extern String receivedCommand; // Befehl von WebSocket
extern String receivedSpeed;   // Geschwindigkeitswert von WebSocket
extern bool isPhysicalControl; // Steuerungsmodus

#endif // GLOBALS_H
