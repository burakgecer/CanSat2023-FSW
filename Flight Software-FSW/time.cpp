/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#include "time.h"

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void teensy_rtc::setup()
{
	setSyncProvider(getTeensy3Time);

  
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  };

}
void teensy_rtc::update()
{
	if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
  digitalClockDisplay();  
  

}
void teensy_rtc::digitalClockDisplay(){
  Serial.print(hour());

  printDigits(minute());
  printDigits(second());

  Serial.print(" ");
//  Serial.print(day());
//  Serial.print(" ");
//  Serial.print(month());
//  Serial.print(" ");
//  Serial.print(year()); 
  Serial.println(); 
}
//time_t teensy_rtc::getTeensy3Time()
//{
//  return Teensy3Clock.get();
//}
unsigned long teensy_rtc::processSyncMessage(){
	unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}
void teensy_rtc::printDigits(int digits){
	// utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

String teensy_rtc::time(){
    return String(hour()) + ":" + String(minute()) + ":" + String(second());
}
