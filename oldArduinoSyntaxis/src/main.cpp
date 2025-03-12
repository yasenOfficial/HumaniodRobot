#include <Arduino.h>
#include <SoftwareSerial.h>
#include "AS5600.h"
#include "MPU9250Driver.h"
#include "DRV8825.h"

SoftwareSerial debug_serial(PA10, PA9);
AS5600 as5600;
MPU9250Driver mpu9250(MPU9250_ADDRESS_AD0_HIGH);
DRV8825 motor(PB4, PB5, PB13, PB15, 200);

void setup() {
    debug_serial.begin(9600);
    debug_serial.println("System starting...");

    // Initialize AS5600
    as5600.begin();

    // Initialize MPU9250
    mpu9250.begin();

    // Initialize Motor
    motor.begin();
}

void loop() {
    // Move motor
    motor.moveMotor();

    // Read and print MPU9250 data
    // mpu9250.updateSensorData();
    // debug_serial.println("Accel X: " + String(mpu9250.getAccelX()));
    // debug_serial.println("Gyro X: " + String(mpu9250.getGyroX()));
    // debug_serial.println("Mag X: " + String(mpu9250.getMagX()));

    // // Magnetic encoder data
    // as5600.readRawAngle();
    // as5600.correctAngle();
    // as5600.checkQuadrant();
    // debug_serial.println("Total Angle (degrees): " + String(as5600.getTotalAngle()));

    // delay(500);
}
