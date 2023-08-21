#pragma once
#include <Arduino.h>

bool sd_setup();
bool write_to_sd(const String &to_write);
