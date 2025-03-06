#ifndef EPIC_DAUG_DEFINES_H
#define EPIC_DAUG_DEFINES_H

// Control pins for the wings stepper drivers
#define STEPPER_PORT_STEP_PIN 3
#define STEPPER_PORT_DIR_PIN 2
#define STEPPER_PORT_SLEEP_PIN 4
#define STEPPER_PORT_RESET_PIN 5
#define STEPPER_PORT_ENABLE_PIN 6

#define STEPPER_STBD_STEP_PIN 37
#define STEPPER_STBD_DIR_PIN 36
#define STEPPER_STBD_SLEEP_PIN 30
#define STEPPER_STBD_RESET_PIN 31
#define STEPPER_STBD_ENABLE_PIN 32

// Pins for the ADC that are reading the elevators hall sensors
#define HALL_A2D_PORT_PIN A12
#define HALL_A2D_STBD_PIN A13

// Potentiometer for controlling thruster
#define POTENTIOMETER_THRUSTER A15
// 1 to send commands to thruster, 0 otherwise
#define USE_THRUSTER 1
#define THRUSTER_CMD_RPM 0
#define THRUSTER_CMD_DUTY 1
#define THRUSTER_CMD_AMPS 2

// Elevator hardware parameters
#define STBD_ELEVATOR_SPEED 600
#define PORT_ELEVATOR_SPEED 600
#define PORT_ELEVATOR_ACCELERATION 500.0
#define STBD_ELEVATOR_ACCELERATION 500.0
#define GEAR_RATIO 256.0//1 for twotrees stepper // Gear reduction ratio between motor and output shaft
#define STEPS_PER_REV 20.0//200 for twotrees stepper // Full steps per revolution for the motor
#define MICROSTEPPING 0.5 // Microstepping selected in h/w, we have MS1 tied to HIGH rail and MS2,MS3 tied to GND
#define DEGREES_PER_REVOLUTION 360.0
#define A2D_SENSOR_RANGE 1023
#define ELEVATOR_MIN_SPEED -600 
#define ELEVATOR_MAX_SPEED 600

// Backseat command types
enum BackseatCommandTypes
{
  NAV,
  CONTROL,
  RESET,
  SET_ELEVATOR_REFERENCE_ANGLE,
  CALIBRATE_ELEVATOR
};

// Struct to hold the polynomial coefficients
struct PolynomialCoefficients3 {
  float a; // Constant term
  float b; // Coefficient for x
  float c; // Coefficient for x^2
  float d; // Coefficient for x^3
};

#endif // EPIC_DAUG_DEFINES_H