#include "MPU9250Driver.h"

MPU9250Driver::MPU9250Driver(uint8_t address, TwoWire &wire) : mpu9250(address), aX(0), aY(0), aZ(0), aSqrt(0), 
    gX(0), gY(0), gZ(0), mX(0), mY(0), mZ(0), mDirection(0) {
    mpu9250.setWire(&wire);
}

void MPU9250Driver::begin() {
    mpu9250.beginAccel();
    mpu9250.beginGyro();
    mpu9250.beginMag();
}

void MPU9250Driver::updateSensorData() {
    if (mpu9250.accelUpdate() == 0) {
        aX = mpu9250.accelX();
        aY = mpu9250.accelY();
        aZ = mpu9250.accelZ();
        aSqrt = mpu9250.accelSqrt();
    }
    if (mpu9250.gyroUpdate() == 0) {
        gX = mpu9250.gyroX();
        gY = mpu9250.gyroY();
        gZ = mpu9250.gyroZ();
    }
    if (mpu9250.magUpdate() != 0) {
        mpu9250.beginMag();
        mpu9250.magUpdate();
    }
    mX = mpu9250.magX();
    mY = mpu9250.magY();
    mZ = mpu9250.magZ();
    mDirection = mpu9250.magHorizDirection();
}

float MPU9250Driver::getAccelX() const { return aX; }
float MPU9250Driver::getAccelY() const { return aY; }
float MPU9250Driver::getAccelZ() const { return aZ; }
float MPU9250Driver::getAccelSqrt() const { return aSqrt; }

float MPU9250Driver::getGyroX() const { return gX; }
float MPU9250Driver::getGyroY() const { return gY; }
float MPU9250Driver::getGyroZ() const { return gZ; }

float MPU9250Driver::getMagX() const { return mX; }
float MPU9250Driver::getMagY() const { return mY; }
float MPU9250Driver::getMagZ() const { return mZ; }
float MPU9250Driver::getMagHorizDirection() const { return mDirection; }
