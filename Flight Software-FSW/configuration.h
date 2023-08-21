/*  
 Copyright (C) 2023 - Seyit Koyuncu and Ekrem Burak Geçer (@burakgecer)
 Flight Software code for CanBee CanSat 2023
*/

#pragma once

#define IS_CONTAINER true
#define IS_PAYLOAD (!(IS_CONTAINER))

#define XBEE_API_MODE true
#define SIM_MODE false

#define xbee_port Serial8

#define TEAM_ID 1008

#include "Arduino.h"
#include "cansat.h"

// Timing library that is included in Teensy.
// See: https://www.pjrc.com/teensy/td_libs_Metro.html
#include <Metro.h>
