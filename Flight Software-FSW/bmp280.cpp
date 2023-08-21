/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#include "bmp280.h"

// Returns true if there was any errors
bool BMP::setup()
{
    status = bmp.begin(0x76, BMP280_CHIPID);
    // status = bmp.begin();
    if (!status)
    {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
        Serial.print("SensorID was: 0x");
        Serial.println(bmp.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        return true;
    }

    /* Default settings from datasheet. */
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
        Adafruit_BMP280::SAMPLING_X2,    /* Temp. oversampling */
        Adafruit_BMP280::SAMPLING_X16,   /* Pressure oversampling */
        Adafruit_BMP280::FILTER_X16,     /* Filtering. */
        Adafruit_BMP280::STANDBY_MS_500  /* Standby time. */
    );
    baseline_pressure = bmp.readPressure() / 100;
    
    
    return false;
}

bool BMP::update() {
    if (!status) return true;
    showed_pressure = bmp.readPressure() / 100;
    temperature = bmp.readTemperature();
    if(!cal_altitude){
          relative_altitude = bmp.readAltitude(baseline_pressure);
    }
    else{
          relative_altitude = bmp.readAltitude(baseline_pressure);

    }

    return false;
}

bool BMP::sim_update(double pressureInPascalUnit) {
    if (!status) return true;
    const auto pressure = pressureInPascalUnit / 100.0;
    if (baseline_pressure != baseline_pressure) { // NaN check
        baseline_pressure = pressure;
    }

    temperature = bmp.readTemperature();
    relative_altitude = ((pow((baseline_pressure/pressure),(1/5.257))-1) * (temperature+273.15))/0.0065;
    showed_pressure = pressure;
    return false;
}

void BMP::CalibrateAltitude()
{     
      baseline_pressure = bmp.readPressure() / 100;
      cal_altitude = true;
}
