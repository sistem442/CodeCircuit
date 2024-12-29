#ifndef WEB_SOCKET_MODULE_H
#define WEB_SOCKET_MODULE_H
#include <Arduino.h> 


void setupWebSocket();
void handleWebSocket();
void updateMotorStatus(String status, String error = ""); // Funktion mit zwei Parametern
void sendSensorAndStatusData();


#endif
