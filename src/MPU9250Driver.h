#ifndef MPU9250DRIVER_H
#define MPU9250DRIVER_H

#include <MPU9250_asukiaaa.h>
#include <Wire.h>

class MPU9250Driver {
public:
    MPU9250Driver(uint8_t address, TwoWire &wire = Wire);
    void begin();
    void updateSensorData();

    float getAccelX() const;
    float getAccelY() const;
    float getAccelZ() const;
    float getAccelSqrt() const;

    float getGyroX() const;
    float getGyroY() const;
    float getGyroZ() const;

    float getMagX() const;
    float getMagY() const;
    float getMagZ() const;
    float getMagHorizDirection() const;

private:
    MPU9250_asukiaaa mpu9250;
    float aX, aY, aZ, aSqrt;
    float gX, gY, gZ;
    float mX, mY, mZ, mDirection;
};

#endif // MPU9250DRIVER_H
