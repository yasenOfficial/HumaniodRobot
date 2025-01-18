#include "DRV8825.h"

DRV8825::DRV8825(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin, uint8_t resetPin, uint16_t stepsPerRevolution)
    : dirPin(dirPin), stepPin(stepPin), enablePin(enablePin), resetPin(resetPin), stepsPerRevolution(stepsPerRevolution) {}

void DRV8825::begin() {
    pinMode(enablePin, OUTPUT);
    pinMode(resetPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    digitalWrite(resetPin, LOW);
    digitalWrite(enablePin, HIGH);
    delay(10);

    digitalWrite(resetPin, HIGH);
    delay(5);
    digitalWrite(enablePin, LOW);
    delay(5);

    digitalWrite(dirPin, LOW);
}

void DRV8825::moveMotor() {
    for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(50);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(50);
    }
}
