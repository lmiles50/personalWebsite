#ifndef EPIC_DAUG_INCLUDES_H
#define EPIC_DAUG_INCLUDES_H

// EEPROM library to load/save values to EEPROM
// https://docs.arduino.cc/learn/built-in-libraries/eeprom
#include <EEPROM.h>

// PID library by Brett Beauregard
// https://www.arduino.cc/reference/en/libraries/pid/
// https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>

// ArduinoJson library by Benoit Blanchon
// https://arduinojson.org/
// https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

// Includes for AccelStepper library (stepper motors)
// https://www.arduino.cc/reference/en/libraries/accelstepper/
// https://github.com/waspinator/AccelStepper
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "epic_daug_nav.h"
#include "vesc_propeller.h"
#include "elevators.h"

#endif // EPIC_DAUG_INCLUDES_H