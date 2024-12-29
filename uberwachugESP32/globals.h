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
extern String motor_status;

#endif // GLOBALS_H
