/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#include "mpu9250.h"

bool MPU::setup() {
    // Note that The specific wire used depends on your PCB design.
    // The MPU9250_asukiaaa we used here might also need some adjustments.
    Wire.begin();
    rawSensor.setWire(&Wire);

    rawSensor.beginAccel();
    return false;
}

bool MPU::update() {
    if (rawSensor.accelUpdate() == 0) {
        accelerometer.roll  = rawSensor.accelX();
        accelerometer.pitch = rawSensor.accelY();
        accelerometer.yaw   = rawSensor.accelZ();

        accelerometer.tilt_x = atan2(-accelerometer.roll, sqrt(accelerometer.pitch*accelerometer.pitch + accelerometer.yaw*accelerometer.yaw)) * 180 / PI;
        accelerometer.tilt_y = atan2(accelerometer.pitch, sqrt(accelerometer.roll*accelerometer.roll + accelerometer.yaw*accelerometer.yaw)) * 180 / PI;

    }
    return false;
}
