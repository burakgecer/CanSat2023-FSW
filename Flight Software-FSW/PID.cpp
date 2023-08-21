/*  
 Copyright (C) 2023 - Samet Efekan Doskaya (@sametefekan)
 Flight Software code for CanBee CanSat 2023
*/

#include "PID.h"
#include <Arduino.h>

float errorIntegral = 0.0;
float previouserrorPosition = 0;
float previousTime = 0;
float numberofTurn = 4.1; 
float target_motor_position = 0;
float encoderRead = 0;
float controlSignal = 0;
float currentTime;
const float Ki = 0.69;
const float Kp = 1.5;
int PWMValue = 0;


static volatile float motorPosition = 0;
static void driveHSMotor();
static void PIDgenerator();
static void PositionGenerator();

void PID::setup()
{
    pinMode(encoderPinDigital, INPUT);
    pinMode(encoderPinInterrupt, INPUT);
    attachInterrupt(
        digitalPinToInterrupt(PID::encoderPinInterrupt), 
        PositionGenerator, 
        RISING);
    target_motor_position = numberofTurn * 1000 * 4 ;
}
void PID::update()
{  
    PIDgenerator();
    driveHSMotor();
}

static void driveHSMotor(){
    PWMValue = (int) fabs(controlSignal);
    if (PWMValue >= 255)
        PWMValue = 255;
    else
        PWMValue = PWMValue;

    
    if(controlSignal <=0)
    {
    analogWrite(PID::openPin,PWMValue);
    digitalWrite(PID::closePin, LOW);
    }
    
    else if (controlSignal >=0)
    {
    analogWrite(PID::closePin,PWMValue);
    digitalWrite(PID::openPin, LOW);
    }

    else if(controlSignal ==0)
    {
    digitalWrite(PID::openPin,LOW);
    digitalWrite(PID::closePin,LOW);
    }
}
void PID::PIDgenerator() {

    currentTime = micros();
    auto deltaTime = (currentTime - previousTime) / 1e6;
    previousTime = currentTime;

    errorPosition = target_motor_position - motorPosition;
         errorIntegral = errorIntegral + (PID::errorPosition + previouserrorPosition)*0.5*deltaTime; 
        if ( errorIntegral >= 50)
               errorIntegral = 50;
        else if ( errorIntegral <= -50)
              errorIntegral = -50;
        else{
               errorIntegral= errorIntegral;
        }
              
    controlSignal = (Kp * PID::errorPosition) + (Ki*errorIntegral);
    previouserrorPosition = PID::errorPosition;
}
void PositionGenerator() {
    encoderRead = digitalRead(PID::encoderPinDigital);

    if (encoderRead == 1) //CW direction
    {
        motorPosition++;
    } else //else, it is zero CCW direction
    {
        motorPosition--;
    }   
}
