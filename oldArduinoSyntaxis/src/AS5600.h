#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
public:
    AS5600(TwoWire &wire = Wire);
    void begin();
    void readRawAngle();
    void correctAngle();
    void checkQuadrant();

    float getDegAngle() const;
    float getCorrectedAngle() const;
    float getTotalAngle() const;

private:
    TwoWire &wire;
    int magnetStatus;
    int lowbyte;
    word highbyte;
    int rawAngle;
    float degAngle;
    float correctedAngle;
    float startAngle;
    float totalAngle;
    float numberofTurns;
    int previousQuadrantNumber;
};

#endif // AS5600_H
