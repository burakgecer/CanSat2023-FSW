/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once

#include <TimeLib.h>
#include <Arduino.h>

#define TIME_HEADER  "T"   // Header tag for serial time sync message

struct teensy_rtc
{
	void setup();
	void update();
	void digitalClockDisplay();
	unsigned long processSyncMessage();
	void printDigits(int digits);
  //time_t getTeensy3Time();
  String time();
};
