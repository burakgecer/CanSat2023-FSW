/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/
#include "configuration.h"
#include "gps_teensy.h"
#include "mpu9250.h"
#include "sd_handler.h"
#include "xbee_handler.h"
#include "cansat.h"
#include <Arduino.h>

Metro packet_timer = Metro(1000);
bool error = false;
unsigned timer_read_data = 0;

Cansat cansat;

void setup() {
    Serial.begin(9600);
    // while (!Serial) {}
    xbee_port.begin(9600); // Container or Payload, see #if above.

    xbee_port.setTimeout(10);
    while (!xbee_port) {}

    sd_setup(); //tekrar bakılacak

    xbee_setup();
    cansat.gps.gps_setup();

    error = cansat.setup();

    while (error) Serial.println("Error in setup!");
}

double altitude_two_secs_ago = 0.0;
double altitude_one_sec_ago = 0.0;

int over_probe_parachute_target_altitude_seconds = 0;
int over_probe_release_altitude_seconds = 0;
int over_heatshield_release_altitude_seconds = 0;



//float previousTime = 0;

void resetTimer() {
  timer_read_data = millis();
}

void loop() {
    cansat.gps.get_gps_readings(); 
    cansat.mpu_data.update();
    
    if(millis() - timer_read_data >= 30)
    {
      auto buffer = getZigBeeData();
      auto cmd = strToCommand(buffer);
    
      if (cmd == CommandType::SimulatedPressureData) {
          auto subst = buffer.substring(4); // assuming SIMP101325
          cansat.lastSimPressure = (double) subst.toInt();
      }
      
      if (cmd != CommandType::None) {
          Serial.println(String("Command: ") + commandToString(cmd));
          cansat.executeCommand(cmd);
          cansat.last_cmd = cmd;
      } 
      resetTimer();   
    }
    
    if (packet_timer.check()) {
        auto err = cansat.update(altitude_one_sec_ago, altitude_two_secs_ago);
        if (err) Serial.println("Error in cansat.update");
        cansat.mission_time += 1;

        // Over parachute control
        {
            if (cansat.bmp.relative_altitude >= cansat.probe_parachute_target_altitude)
                over_probe_parachute_target_altitude_seconds += 1;
            else if ((cansat.bmp.relative_altitude < cansat.probe_parachute_target_altitude) && (!cansat.over_probe_parachute_target_altitude))
                over_probe_parachute_target_altitude_seconds = 0;

            if (over_probe_parachute_target_altitude_seconds > 3)
                cansat.over_probe_parachute_target_altitude = true;
        }

        // Over payload control
        {
            if (cansat.bmp.relative_altitude >= cansat.probe_release_altitude)
                over_probe_release_altitude_seconds += 1;
            else if ((cansat.bmp.relative_altitude < cansat.probe_release_altitude) && (!cansat.over_probe_release_altitude))
                over_probe_release_altitude_seconds = 0;

            if (over_probe_release_altitude_seconds > 3)
                cansat.over_probe_release_altitude = true;
        }

        //Over Heatshield control
        {
            if (cansat.bmp.relative_altitude >= cansat.heatshield_release_altitude)
                over_heatshield_release_altitude_seconds += 1;
            else if ((cansat.bmp.relative_altitude < cansat.heatshield_release_altitude) && (!cansat.over_heatshield_release_altitude))
                over_heatshield_release_altitude_seconds = 0;

            if (over_heatshield_release_altitude_seconds > 3)
                cansat.over_heatshield_release_altitude = true;
        }
        
        const auto telemetry = cansat.payloadTelemetryStr(cansat.last_cmd);
        
        if (cansat.sendTelemetry && (telemetry.length() > 4))
        {
            cansat.packet_count += 1;
            Serial.print("Gonderilen telemetri: ");
            Serial.println(telemetry);
            sendData(telemetry, GROUND_CS_ADDRESS);
            write_to_sd(telemetry);
        }

        altitude_two_secs_ago = altitude_one_sec_ago;
        altitude_one_sec_ago  = cansat.bmp.relative_altitude;
    }
}
