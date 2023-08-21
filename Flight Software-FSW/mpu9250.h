/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once

#include <MPU9250_asukiaaa.h>
struct MPU {
    struct MPUReading {
        //Added tilt_x and tilt_y
        float roll, pitch, yaw, tilt_x, tilt_y;
    };

    MPUReading accelerometer;

    bool setup();
    bool update();
    
    MPU9250_asukiaaa rawSensor;
};
