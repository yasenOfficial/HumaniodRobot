#include "AS5600.h"

AS5600::AS5600(TwoWire &wire) : wire(wire), magnetStatus(0), lowbyte(0), highbyte(0),
    rawAngle(0), degAngle(0), correctedAngle(0), startAngle(0), totalAngle(0),
    numberofTurns(0), previousQuadrantNumber(0) {}

void AS5600::begin() {
    wire.begin();
}

void AS5600::readRawAngle() {
    wire.beginTransmission(0x36);
    wire.write(0x0D);
    wire.endTransmission();

    wire.requestFrom(0x36, 1);
    while (wire.available() == 0);
    lowbyte = wire.read();

    wire.beginTransmission(0x36);
    wire.write(0x0C);
    wire.endTransmission();
    wire.requestFrom(0x36, 1);
    while (wire.available() == 0);
    highbyte = wire.read();

    highbyte = highbyte << 8;
    rawAngle = highbyte | lowbyte;
    degAngle = rawAngle * 0.087890625;
}

void AS5600::correctAngle() {
    correctedAngle = degAngle - startAngle;
    if (correctedAngle < 0) {
        correctedAngle += 360;
    }
}

void AS5600::checkQuadrant() {
    int quadrantNumber;
    if (correctedAngle >= 0 && correctedAngle <= 90) quadrantNumber = 1;
    else if (correctedAngle > 90 && correctedAngle <= 180) quadrantNumber = 2;
    else if (correctedAngle > 180 && correctedAngle <= 270) quadrantNumber = 3;
    else quadrantNumber = 4;

    if (quadrantNumber != previousQuadrantNumber) {
        if (quadrantNumber == 1 && previousQuadrantNumber == 4) numberofTurns++;
        if (quadrantNumber == 4 && previousQuadrantNumber == 1) numberofTurns--;
        previousQuadrantNumber = quadrantNumber;
    }

    totalAngle = (numberofTurns * 360) + correctedAngle;
}

float AS5600::getDegAngle() const {
    return degAngle;
}

float AS5600::getCorrectedAngle() const {
    return correctedAngle;
}

float AS5600::getTotalAngle() const {
    return totalAngle;
}
