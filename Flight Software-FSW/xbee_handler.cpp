// Copyright 2020 - 2022 akgvn
// Some parts contributed by Berkay Apalı

#include "configuration.h"
#include "xbee_handler.h"

// I don't like this - ag
XBee xbee = XBee();

void xbee_setup() {
    xbee.begin(xbee_port);
}

String getZigBeeData() {
    xbee.readPacket();
    auto response = xbee.getResponse();

    if (response.isAvailable() && response.getApiId() == ZB_RX_RESPONSE) {
        auto rx = ZBRxResponse();
        xbee.getResponse().getZBRxResponse(rx);

        uint8_t len   = rx.getDataLength();
        uint8_t* data = rx.getData(); // This may be accessed from index 0 to getDataLength() - 1

        auto resultString = String("");

        for (size_t idx = 0; idx < len; idx++) {
            auto character_uint = data[idx];
            if (character_uint != 0)
                resultString += (char)character_uint;
            else
                break;
        }

        return resultString;
    }
    return String("");
}

void sendData(const String &payload, XBeeAddress64 address) {
    // TODO const char* to uint8_t* conversion warning!
    const uint8_t* raw_payload = (uint8_t*) payload.c_str();
    const uint8_t  payload_length = payload.length();

    auto txReq = ZBTxRequest(address, raw_payload, payload_length);
    xbee.send(txReq);
}

// Command stuff
// The following is why it is not fun to use C++'s enum classes.

String commandToString(CommandType cmd) {
    switch (cmd) {
        case CommandType::None:                  return String("None");
        case CommandType::StartTelemetry:        return String("ST");
        case CommandType::StopTelemetry:         return String("StopTelemetry");
        case CommandType::StartPayloadTelemetry: return String("StartPayloadTelemetry");
        case CommandType::StopPayloadTelemetry:  return String("StopPayloadTelemetry");
        case CommandType::StartSimulationMode:   return String("StartSimulationMode");
        case CommandType::ActivateSimulationMode: return String("SimulationActivated");
        case CommandType::StopSimulationMode:    return String("StopSimulationMode");
        case CommandType::SimulatedPressureData: return String("SimulatedPressureData");
        case CommandType::StartCamera:           return String("StartCamera");
        case CommandType::StopCamera:            return String("StopCamera");
        case CommandType::StartBuzzer:           return String("StartBuzzer");
        case CommandType::StopBuzzer:            return String("StopBuzzer");
        case CommandType::ReleaseParachute:      return String("ReleaseParachute");
        case CommandType::ReleasePayload:        return String("ReleasePayload");

        case CommandType::Calibration:           return String("Calibration");               

        case CommandType::SetTime:               return String("TimeSetted");               

        case CommandType::Parachute90:           return String("P90");
        case CommandType::Parachute0:            return String("P0");

        case CommandType::Release90:             return String("R90");
        case CommandType::Release0:              return String("R0");

        case CommandType::ReleaseHeatShield:     return String("RHS");

        case CommandType::RaiseFlag:            return String("RFS");
        case CommandType::Close_Flag:           return String("CF");

        case CommandType::Upright_Heatshield:    return String("UHS");
        case CommandType::Close_Heatshield:      return String("CHS");

        
    }
    return String("Error in commandToString!");
}

CommandType strToCommand(const String &incoming_packet)
{
    if (incoming_packet.indexOf("CMD,1008,CX,ON")   != -1) return CommandType::StartTelemetry;
    if (incoming_packet.indexOf("CMD,1008,CX,OFF")  != -1) return CommandType::StopTelemetry;

    if (incoming_packet.indexOf("CMD,1008,CAL")     != -1) return CommandType::Calibration;

    if (incoming_packet.indexOf("CMD,1008,ST")      != -1) return CommandType::SetTime;

    if (incoming_packet.indexOf("CMD,1008,RLS90") != -1)  return CommandType::Release90;
    if (incoming_packet.indexOf("CMD,1008,RLS0")  != -1)  return CommandType::Release0;

    if (incoming_packet.indexOf("CMD,1008,PRC90") != -1)  return CommandType::Parachute90;
    if (incoming_packet.indexOf("CMD,1008,PRC0")  != -1)  return CommandType::Parachute0;

    if (incoming_packet.indexOf("CMD,1008,RHS")   != -1)  return CommandType::ReleaseHeatShield;

    if (incoming_packet.indexOf("CMD,1008,RFS")   != -1) return  CommandType::RaiseFlag;
    if (incoming_packet.indexOf("CMD,1008,CF" )   != -1)  return CommandType::Close_Flag;

    if (incoming_packet.indexOf("CMD,1008,UHS")   !=-1)  return CommandType::Upright_Heatshield;
    if (incoming_packet.indexOf("CMD,1008,CHS")    !=-1)  return CommandType::Close_Heatshield;

    if (incoming_packet.indexOf("CMD,1008,CAM,ON")  != -1) return CommandType::StartCamera;
    if (incoming_packet.indexOf("CMD,1008,CAM,OFF") != -1) return CommandType::StopCamera;

    if (incoming_packet.indexOf("CMD,1008,BUZ,ON")  != -1) return CommandType::StartBuzzer;
    if (incoming_packet.indexOf("CMD,1008,BUZ,OFF") != -1) return CommandType::StopBuzzer;

    if (incoming_packet.indexOf("CMD,1008,SIM,ENABLE")  != -1)  return CommandType::StartSimulationMode;
    if (incoming_packet.indexOf("CMD,1008,SIM,ACTIVATE") != -1) return CommandType::ActivateSimulationMode;
    if (incoming_packet.indexOf("CMD,1008,SIM,DISABLE") != -1)  return CommandType::StopSimulationMode;

    if (incoming_packet.indexOf("SIMP") != -1)    return CommandType::SimulatedPressureData;
    
    return CommandType::None;
}
