/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

struct BMP {
    double temperature, relative_altitude, baseline_pressure, showed_pressure;
    unsigned status;
    bool cal_altitude = false;
    // The wire depends on the PCB!!!
    Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire); // I2C

    bool setup();
    bool update();
    bool sim_update(double pressure);
    void CalibrateAltitude();
};
