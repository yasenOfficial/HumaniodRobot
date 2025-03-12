#ifndef DRV8825_H
#define DRV8825_H

#include <Arduino.h>

class DRV8825 {
public:
    DRV8825(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin, uint8_t resetPin, uint16_t stepsPerRevolution);
    void begin();
    void moveMotor();

private:
    uint8_t dirPin, stepPin, enablePin, resetPin;
    uint16_t stepsPerRevolution;
};

#endif // DRV8825_H
