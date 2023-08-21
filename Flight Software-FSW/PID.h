/*  
 Copyright (C) 2023 - Samet Efekan Doskaya (@sametefekan)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once

struct PID{
    static const int encoderPinDigital = 30;
    static const int encoderPinInterrupt = 31;

    static const int closePin = 28;
    static const int openPin = 29;

    float errorPosition = 0;

    void setup();
    void update();
    void PIDgenerator();

    


    
        






};
