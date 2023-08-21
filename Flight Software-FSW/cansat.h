/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once

#include <Servo.h>
#include <Metro.h>
#include "configuration.h"
#include "MPU9250.h"
#include "gps_teensy.h"
#include "xbee_handler.h"
#include "bmp280.h"
#include "PID.h"
#include "time.h"

struct Cansat {
    enum class Mode {
        Flight     = 'F',
        Simulation = 'S'
    };

    enum class PacketType {
        Payload   = 'T',
        Custom    = 'X'
    };

    enum class FlightState {
        SensorsOff,
        BeforeLaunch,
        Ascent,
        Descent,
        ReleasePayload,
        ReleaseHeatShield,
        ReleaseParachute,
        OneSec,
        Landing,
        Uprighting,
        Flag
    };

    struct CustomServo {
        Servo servo_motor;
        int angle;
        Metro servo_timer;
        CustomServo(): servo_timer(Metro(0)) {}

        void setupServo(int servo_pin, int angle);
        bool update(int angle);
    };

    const unsigned int team_id = TEAM_ID;
    unsigned int mission_time = 0;
    unsigned int packet_count;
    PacketType type;
    FlightState state;
    float altitude;
    float temperature;
    float voltage;
    bool sendTelemetry = false;
    Mode mode;
    String hs_deployed = "N" ;
    String pc_deployed = "N" ;
    String mast_raised = "N" ;
    double lastSimPressure = sqrt(-1); // NaN

    CommandType last_cmd = CommandType::None;

    int countrhs = 0;

    Metro camera_timer_on = Metro(1000);
    Metro camera_timer_off = Metro(1000);
    
    const byte CAM_HIGH_PIN = 23;
    const byte CAM_TRIG_PIN = 7; 
    void setup_cam();
    void toggle_camera_recording();
    void stop_camera_recording();

    const byte BUZZER_PIN = 22;
    void enable_buzzer()  { pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); }
    void disable_buzzer() { pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);  }

    const byte analogInputPin = 14; // Container 14. pin

    MPU mpu_data;
    float pointing_error;

    String payloadTelemetryStr(CommandType cmd) const;

    BMP bmp;
    GPSData gps;
    teensy_rtc clock_time;

    PID pid;

    CustomServo parachuteServo;
    CustomServo payloadServo;

    bool parachute_released;
    bool probe_released;
    bool heatshiled_released = false;
    bool uprighting_bool = false;
    bool flag_release;
    bool is_landing = false;

    unsigned long telemetry_send_while = 0; 
    unsigned long previousTime = 0;
    //unsigned long raiseDelay = 2000;
    //unsigned long currentTime = millis();

    //---------FLAG----------
    static const int ShortenPin = 36;
    static const int RaisePin = 37;
    unsigned long previousTimeFlag = 0;
    unsigned long raiseDelay = 5000;
    bool raiseFlag = false;
    
    //-------UPRIGHTING--------
    static const int UprightClosePin = 29;
    static const int UprightOpenPin = 28;
    bool upright_heatshield = false;
    unsigned long previousTimeUpright = 0;
    unsigned long UprightDelay = 30000;

    //--------ANGLE-30 HEATSHIELD------------
    bool open_heatshield = false;
    unsigned long previousTimeHeatshield = 0;
    unsigned long HeatshieldDelay = 7000;

    //-------ONE-SEC HEATSHIELD----------
    bool one_sec_heatshield = false;
    unsigned long previousTimeOneSec = 0;
    unsigned long OneSecDelay = 3000;

    //------CLOSE FLAG--------
    bool closeFlag = false;

    //------Open FLAG Air--------
    unsigned long FlagOpenInAir = 250;
    bool closeFlagAir = false;
    

    //bunlarin ne zaman true olacagina bakmadim daha, ozellikte heatshield olan
    bool over_probe_parachute_target_altitude = false;
    bool over_probe_release_altitude = false;
    bool over_heatshield_release_altitude = false;
    
    static const int probe_parachute_target_altitude = 200; // in meters //Deploy probe's parachute
    static const int probe_release_altitude = 500;  // in meters //also open heat shield after release probe 
    static const int heatshield_release_altitude = 475; //450 yi farazi yazdim probe salindiktan hemen sonra kac metrede olursa onunla degistircez
    
    String stateStr() const;
    String modeStr() const;

    void Activate_Uprighting(); //Method for uprighting
    void Raise_Flag(); //Method for raise flag
    void Release_Heatshield();
    void Upright_Heatshield();
    void Close_Heatshield();
    void Close_Flag();
    void Open_Flag_Air();

    bool setup();
    bool update(double altitude_one_sec_ago, double altitude_two_secs_ago);
    void executeCommand(CommandType cmd);

    void voltmeter_setup() { pinMode(analogInputPin, INPUT); }
    float get_voltage(); 
};
