/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#include "cansat.h"
#include "Arduino.h"

static bool bmpMovingAverage(BMP &bmp) {
    double bmp_avg = 0.0;
    int sample_size = 5;
    bool err = false;
    for (int i = 0; i < sample_size; i++)
    {
        err = bmp.update();
        bmp_avg += bmp.relative_altitude;
    }
    bmp_avg /= sample_size;
    bmp.relative_altitude = bmp_avg;

    return err;
}

static bool isAscending(
    double current_altitude,
    double altitude_one_sec_ago,
    double altitude_two_secs_ago
    
) {
    
    return ((current_altitude - altitude_one_sec_ago) > 8.0)
        && ((altitude_one_sec_ago - altitude_two_secs_ago) > 8.0);
}

static bool isDescending(
    double current_altitude,
    double altitude_one_sec_ago,
    double altitude_two_secs_ago
) {
    return ((current_altitude - altitude_one_sec_ago) < -8.0)
        && ((altitude_one_sec_ago - altitude_two_secs_ago) < -8.0);
}

static bool isLanding(
    double current_altitude,
    double altitude_one_sec_ago,
    double altitude_two_secs_ago
) {
    Serial.println(current_altitude);
    Serial.println(altitude_one_sec_ago);
    Serial.println(altitude_two_secs_ago);
    return ((current_altitude - altitude_one_sec_ago) < 1.0)
        && ((altitude_one_sec_ago - altitude_two_secs_ago) < 1.0);
}


String Cansat::stateStr() const {
    switch (state) {
        case Cansat::FlightState::SensorsOff:       return String("SOff");
        case Cansat::FlightState::BeforeLaunch:     return String("BeforeLaunch");
        case Cansat::FlightState::Ascent:           return String("Ascent");
        case Cansat::FlightState::Descent:          return String("Descent");
        case Cansat::FlightState::ReleasePayload:   return String("ReleasePayload");
        
        case Cansat::FlightState::ReleaseHeatShield: return String("ReleaseHeatShield");
        case Cansat::FlightState::ReleaseParachute: return String("ReleaseParachute");
        case Cansat::FlightState::OneSec:           return String("OneSecHS");
        case Cansat::FlightState::Landing:          return String("Landing");
        case Cansat::FlightState::Uprighting:       return String("Uprighting");
        case Cansat::FlightState::Flag:             return String("Flag");
    }
    return String("Error in stateStr!");
}

static const String comma = String(",");

//#if IS_PAYLOAD
String Cansat::payloadTelemetryStr(CommandType cmd) const {
    // <TEAM_ID>, <MISSION_TIME>, <PACKET_COUNT>, <MODE>, <STATE>,
    // <ALTITUDE>, <HS_DEPLOYED>, <PC_DEPLOYED>, <MAST_RAISED>, <TEMPERATURE>, <PRESSURE>, <VOLTAGE>,
    // <GPS_TIME>, <GPS_ALTITUDE>, <GPS_LATITUDE>, <GPS_LONGITUDE>, <GPS_SATS>, <TILT_X>, <TILT_Y>, <CMD_ECHO>
    
    auto pyld = (probe_released) ? String("1") : String("0");
    auto packet = String(team_id)                     +
        comma + String(clock_time.time())                  +
        comma + String(packet_count)                  +
        comma + String(modeStr())                     +
        comma + String(stateStr())                    +
        comma + String(altitude)                      +
        comma + String(hs_deployed)                   + //HS_DEPLOYED variablei ayarlanacak //‘P’ indicates the Probe with heat shield is deployed, ‘N’ otherwise.
        comma + String(pc_deployed)                   + //‘C’ indicates the Probe parachute is deployed (at 200 m), ‘N’ otherwise
        comma + String(mast_raised)                   + //‘M’ indicates the flag mast has been raised after landing, ‘N’ otherwise
        comma + String(temperature)                   +
        comma + String(bmp.showed_pressure)           +
        comma + String(voltage)                       +
        comma + String(gps.time())                    +
        comma + String(gps.altitude)                  +
        comma + String(gps.latitude / 100)                  +
        comma + String(gps.longtitude / -100)                +
        comma + String(gps.satellites)                +
        comma + String(mpu_data.accelerometer.tilt_x) +
        comma + String(mpu_data.accelerometer.tilt_y) +
        //comma + String(mpu_data.accelerometer.pitch)  + //for vibration
        //comma + String(mpu_data.accelerometer.roll)   + //for vibration
        //comma + String(mpu_data.accelerometer.yaw)    + //for vibration
        comma + commandToString(cmd);
     //telemetry_send_while = millis();
    Serial.println(packet);
    return packet;
}
//#endif

bool Cansat::setup() {
    state = Cansat::FlightState::SensorsOff;
    auto err = false;
    mode = Mode::Flight;

    parachute_released = false;
    probe_released = false;
    heatshiled_released = false;
    
    //For Release
    payloadServo.setupServo(12, 106);
    //payloadServo.servo_motor.write(130);

    //Parachute
    parachuteServo.setupServo(25, 0);
    //parachuteServo.servo_motor.write(75);

    //UPDATE SERVO ANGLES FOR SETUP

    setup_cam();
    disable_buzzer();
    
    err = mpu_data.setup() || err;

    clock_time.setup();

    Serial.println("After MPU err is");
    Serial.println(err);

    voltmeter_setup();
    pid.setup();

    err = bmp.setup() || err;
    Serial.println("After BMP err is");
    Serial.println(err);

    pinMode(RaisePin, OUTPUT);
    pinMode(ShortenPin, OUTPUT);
    
    return err;
}
bool Cansat::update(double altitude_one_sec_ago, double altitude_two_secs_ago) {
    bool error = false;
    error = bmp.update();
    clock_time.update();
    bool ascending  = isAscending(bmp.relative_altitude, altitude_one_sec_ago, altitude_two_secs_ago);
    bool descending = isDescending(bmp.relative_altitude, altitude_one_sec_ago, altitude_two_secs_ago);
    
    if (state > Cansat::FlightState::SensorsOff)
    {
        sendTelemetry = true;
    }
      
    if ((ascending) && (state <= Cansat::FlightState::BeforeLaunch))
        state = Cansat::FlightState::Ascent;

    if ((descending) && (state == Cansat::FlightState::Ascent))
        state = Cansat::FlightState::Descent;

    if ((over_probe_release_altitude) && (state == Cansat::FlightState::Descent) && (altitude <= probe_release_altitude)) {
        state = Cansat::FlightState::ReleasePayload;
        probe_released = payloadServo.update(3);
        toggle_camera_recording();
    }

    if((over_heatshield_release_altitude) && (state == Cansat::FlightState::ReleasePayload) && (altitude <= heatshield_release_altitude))
    { 
      //heatshiled_released = true;
      hs_deployed = "P";
      Release_Heatshield();
      state = Cansat::FlightState::ReleaseHeatShield;
      }

    if ((over_probe_parachute_target_altitude) && (state == Cansat::FlightState::ReleaseHeatShield) && (altitude <= probe_parachute_target_altitude)) {
        Open_Flag_Air();
        state = Cansat::FlightState::ReleaseParachute;
        parachute_released = parachuteServo.update(180);
        pc_deployed = "C";
    }
  
    if ((parachute_released) && (probe_released) && (open_heatshield) && (state == Cansat::FlightState::ReleaseParachute) && (altitude <= 100)){
      state = Cansat::FlightState::OneSec; 
      Activate_Uprighting(); 
    }
      
      if ((parachute_released) && (probe_released) && (open_heatshield) && (state == Cansat::FlightState::OneSec) && (altitude <= 25)){
      toggle_camera_recording();
      state = Cansat::FlightState::Landing;    
    }
      
    if (state == Cansat::FlightState::Landing) {
      is_landing = isLanding(altitude, altitude_one_sec_ago, altitude_two_secs_ago);
      if(is_landing)
      {
          mast_raised = "M";    
          uprighting_bool = true;
          if(uprighting_bool)
          {
              state = Cansat::FlightState::Uprighting;
              
          }         
      }
  }
    if(uprighting_bool && state == Cansat::FlightState::Uprighting)
    {
      Serial.println("GİRDİ");
      Upright_Heatshield();
      Raise_Flag();
      state=Cansat::FlightState::Flag;
      enable_buzzer();
    } 
    
#if IS_CONTAINER
    if (mode == Mode::Simulation)
        error = bmp.sim_update(lastSimPressure);
    else
        error = bmpMovingAverage(bmp);

    if (error) Serial.println("Error updating bmp");

    altitude = bmp.relative_altitude;
    temperature = bmp.temperature;
#endif

    if (error) Serial.println("Error updating gps");
    voltage = get_voltage();
    return error;
}

void Cansat::setup_cam() {
    pinMode(CAM_HIGH_PIN, OUTPUT);
    digitalWrite(CAM_HIGH_PIN, HIGH);
    Serial.println("Setup cam yapti");
}

// TODO Async waits
void Cansat::toggle_camera_recording() {
     pinMode(CAM_TRIG_PIN, OUTPUT);
     digitalWrite(7,LOW);
     delay(1000);
     digitalWrite(7,HIGH);  
}


void Cansat::CustomServo::setupServo(int servo_pin, int angle) {
    Serial.println("SERVO SETUP WORKING");
    servo_motor.attach(servo_pin);
    servo_motor.write(angle);
    Serial.println(angle);
}

bool Cansat::CustomServo::update(int angle) {
    servo_motor.write(angle);
    return true;
}


float Cansat::get_voltage()
{
    float analog_value = analogRead(analogInputPin); 
    float vout = (analog_value/411) * 8.50;    
    return vout;
}

void Cansat::executeCommand(CommandType cmd) {
#if IS_PAYLOAD
    if (cmd == CommandType::StartPayloadTelemetry)
        sendTelemetry = true; // TODO for payload
    if (cmd == CommandType::StopPayloadTelemetry)
        sendTelemetry = false; // TODO for payload
#endif

#if IS_CONTAINER
    if (cmd == CommandType::StartCamera || cmd == CommandType::StopCamera)
      toggle_camera_recording();
      
    if (cmd == CommandType::Parachute90)
        parachuteServo.servo_motor.write(180);
        
    if (cmd == CommandType::Parachute0)
        parachuteServo.servo_motor.write(0);
        
    if (cmd == CommandType::Release90) 
        payloadServo.servo_motor.write(3);
        
    if (cmd == CommandType::Release0) 
        payloadServo.servo_motor.write(106);
        
    if (cmd == CommandType::ReleaseHeatShield)
    {
      while(1){
        Release_Heatshield();
      }
    }
    
    if (cmd == CommandType::RaiseFlag){
      while(1){
        Raise_Flag();
      }
    }
    
    if (cmd == CommandType::Upright_Heatshield){
      while(1){
        Upright_Heatshield();
      }
    }
    
    if (cmd == CommandType::Close_Heatshield){
      while(1){
        Close_Heatshield();
        }
      }
      
     if (cmd== CommandType::Close_Flag){
      while(1){
        Close_Flag();
      }
     }
     
     if(cmd == CommandType::Calibration)
     {
        bmp.CalibrateAltitude();
      }
      
    if (cmd == CommandType::SetTime){
      gps.set_gpsTime();
      
    }
        
    if (cmd == CommandType::StartBuzzer)
        enable_buzzer();
        
    if (cmd == CommandType::StopBuzzer)
        disable_buzzer();
        
    if (cmd == CommandType::ReleasePayload)
        probe_released = true;
        
    if (cmd == CommandType::ReleaseParachute)
        parachute_released = true;
        
    if (cmd == CommandType::StartTelemetry)
        sendTelemetry = true;
        
    if (cmd == CommandType::StopTelemetry)
        sendTelemetry = false;
        
    if (cmd == CommandType::StartSimulationMode) {
        mode = Mode::Simulation;
        bmp.baseline_pressure = sqrt(-1); // NaN
    }
    
    if (cmd == CommandType::ActivateSimulationMode) {
        mode = Mode::Simulation;
        bmp.baseline_pressure = sqrt(-1); // NaN
    }
    
    if (cmd == CommandType::StopSimulationMode)
        mode = Mode::Flight;

#endif
}


String Cansat::modeStr() const {
    if (mode == Cansat::Mode::Flight)     return String("F");
    if (mode == Cansat::Mode::Simulation) return String("S");
    return String("Error in modeStr");
}

void Cansat::Activate_Uprighting()
{ 
  telemetry_send_while = millis();

  Serial.println("HEATSHIELD");
  digitalWrite(UprightOpenPin, LOW);
  digitalWrite(UprightOpenPin, HIGH);
  digitalWrite(UprightClosePin, LOW);
  unsigned long currentTimeOneSec = millis();
  while(!one_sec_heatshield){
    if(millis() - telemetry_send_while >= 1000)
       {
          const auto telemetry = payloadTelemetryStr(last_cmd);
          bool error;
          error = bmp.update();
          gps.get_gps_readings();
          mpu_data.update();
          if (sendTelemetry && (telemetry.length() > 4))
          {
              packet_count += 1;
              Serial.print("Gonderilen telemetri: ");
              Serial.println(telemetry);
              sendData(telemetry, GROUND_CS_ADDRESS);
              //write_to_sd(telemetry);
              
          }  
          telemetry_send_while = millis();
       }
  if (!one_sec_heatshield && (millis() - currentTimeOneSec >= OneSecDelay)) {
      Serial.println("IF'e girdi");

      digitalWrite(UprightOpenPin, LOW); // Set the RaisePin to LOW
      one_sec_heatshield = true;
      }
    }
  return true;
}

void Cansat::Release_Heatshield()
{
  telemetry_send_while = millis();

  Serial.println("HEATSHIELD");
  digitalWrite(UprightOpenPin, LOW);
  digitalWrite(UprightOpenPin, HIGH);
  digitalWrite(UprightClosePin, LOW);
  unsigned long currentTimeHeatshield = millis();
  while(!open_heatshield){
    if(millis() - telemetry_send_while >= 1000)
       {
          const auto telemetry = payloadTelemetryStr(last_cmd);
          bool error;
          error = bmp.update();
          gps.get_gps_readings();
          mpu_data.update();
          clock_time.update();
          if (sendTelemetry && (telemetry.length() > 4))
          {
              packet_count += 1;
              Serial.print("Gonderilen telemetri: ");
              Serial.println(telemetry);
              sendData(telemetry, GROUND_CS_ADDRESS);
              //write_to_sd(telemetry);
              
          }  
          telemetry_send_while = millis();
       }
  if (!open_heatshield && (millis() - currentTimeHeatshield >= HeatshieldDelay)) {
      Serial.println("IF'e girdi");

    digitalWrite(UprightOpenPin, LOW); // Set the RaisePin to LOW
    open_heatshield = true;
      }
    }
}
  

void Cansat::Close_Heatshield(){
  digitalWrite(UprightClosePin, LOW);
  digitalWrite(UprightClosePin, HIGH);
  digitalWrite(UprightOpenPin, LOW);
  unsigned long currentTimeUpright = millis();
  while(!upright_heatshield){
      Serial.println("While");
  if (!upright_heatshield && (millis() - currentTimeUpright >= UprightDelay)) {
      Serial.println("IF'e girdi");

    digitalWrite(UprightClosePin, LOW); // Set the RaisePin to LOW
    upright_heatshield = true;
      }
    }
}
  

void Cansat::Upright_Heatshield(){
  Serial.println("UPRIGHT");
  digitalWrite(UprightOpenPin, LOW);
  digitalWrite(UprightOpenPin, HIGH);
  digitalWrite(UprightClosePin, LOW);
  unsigned long currentTimeUpright = millis();
  while(!upright_heatshield){
      Serial.println("AAAAAAAAAAAAAAAAAAAA");
  if (!upright_heatshield && (millis() - currentTimeUpright >= UprightDelay)) {
      Serial.println("IF'e girdi");

    digitalWrite(UprightOpenPin, LOW); // Set the RaisePin to LOW
    upright_heatshield = true;
      }
    }
}

void Cansat::Raise_Flag()
{
  Serial.println("Raise_Flag() Girdi");
  digitalWrite(RaisePin, HIGH);
  digitalWrite(ShortenPin, LOW);
  unsigned long currentTime = millis();
  while(!raiseFlag){
      Serial.println("While");
  if (!raiseFlag && (millis() - currentTime >= raiseDelay)) {
      Serial.println("IF'e girdi");

    digitalWrite(RaisePin, LOW); // Set the RaisePin to LOW
    raiseFlag = true;
      }
    }
  return;
}

void Cansat::Open_Flag_Air()
{
  Serial.println("Raise_Flag_Air Girdi");
  digitalWrite(RaisePin, HIGH);
  digitalWrite(ShortenPin, LOW);
  unsigned long currentTime = millis();
  while(!closeFlagAir){
  if (!closeFlagAir && (millis() - currentTime >= FlagOpenInAir)) {

    digitalWrite(RaisePin, LOW); // Set the RaisePin to LOW
    closeFlagAir = true;
      }
    }
  return;
}

void Cansat::Close_Flag()
{
  Serial.println("Raise_Flag() Girdi");
  digitalWrite(ShortenPin, HIGH);
  digitalWrite(RaisePin, LOW);
  unsigned long currentTime = millis();
  while(!closeFlag){
      Serial.println("While");
  if (!closeFlag && (millis() - currentTime >= raiseDelay)) {
      Serial.println("IF'e girdi");

    digitalWrite(ShortenPin, LOW); // Set the RaisePin to LOW
    closeFlag = true;
      }
    }
  return;
}
