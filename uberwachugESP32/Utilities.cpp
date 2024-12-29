#include "Utilities.h"
#include <Arduino.h>
#include "globals.h"

bool debounce(int pin, unsigned long& lastDebounceTime, unsigned long debounceDelay) {
    bool reading = digitalRead(pin);
    if (reading == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = millis();
        return true;
    }
    return false;
}
