/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once
#include <Adafruit_GPS.h>
// what's the name of the hardware serial port?

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

struct GPSData {
  
    double  latitude, longtitude, altitude; // altitude is in meters.
    uint8_t hour, minute, second;
    uint32_t satellites;
    
    uint32_t timer = millis();
    
    
    String time() const;
    void gps_setup();
    void get_gps_readings();
    bool set_time = false;
    void set_gpsTime();
};
