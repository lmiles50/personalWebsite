#ifndef ELEVATORS_H
#define ELEVATORS_H

#include "epic_daug_defines.h"

// Includes for AccelStepper library (stepper motors)
// https://www.arduino.cc/reference/en/libraries/accelstepper/
// https://github.com/waspinator/AccelStepper
#include <AccelStepper.h>
#include <MultiStepper.h>

// PID library by Brett Beauregard
// https://www.arduino.cc/reference/en/libraries/pid/
// https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>

#include "vesc_propeller.h"

enum ElevatorType
{
  STBD,
  PORT
};

class Elevator
{
  public:
    enum ControlMode
    {
      OPEN_LOOP,
      HALL_PID
    };

    typedef struct
    {
      int min;
      int max;
    } HallLimits;

    typedef struct
    {
      int min;
      int max;
    } AngleLimits;

    typedef struct
    {
      double angle;
      Elevator::ControlMode mode;
      uint8_t hallPin;
      uint8_t resetPin;
      uint8_t sleepPin;
      double hallScaleDegPerBit;
      int speed;
      int maxSpeed;
      float acceleration;
      double input;
      double output;
      double setpoint;
      float stepsPerDegree;
      int hall; 
      int distanceToGo;
      int currentPosition;
      float openLoopReferenceAngle;
      float pidReferenceAngle;
	  } info;

    Elevator(uint8_t step, uint8_t dir, uint8_t enable, uint8_t hall_pin, int speed, int max_speed, float acceleration, int sleep_pin, int reset_pin, ElevatorType type);

    // This must be called as frequently as possible by the main loop
    void run();

    // Sets the dynamics of the drive
    void setDynamics(double speed, double max_speed, double acceleration);

    // 0: steps only; 1: steps+feedback PID
    void setMode(Elevator::ControlMode mode_in);

    // Set an angle value (degrees) from a feedback sensor, i.e. Hall potentiometer
    void setAngleGoal(double angle_in);

    // Set an position value
    void setPositionGoal(int position);

    // Get info for this elevator
    Elevator::info getInfo();

    // Run to position corresponding to hall value
    void runToHall(int hall);

    // Method to trigger calibration
    void calibrate();

    // Convert current position to angle and save as the reference angle
    void setReferenceAngle();

    // Calculate 3rd order polynomial
    float calculatePolynomial3(float x, PolynomialCoefficients3 poly3);

  private:
    double angle;
    Elevator::ControlMode mode;
    uint8_t hallPin;
    uint8_t resetPin;
    uint8_t sleepPin;
    double hallScaleDegPerBit;
    int speed;
    int maxSpeed;
    float acceleration;
    double input;
    double output;
    double setpoint;
    float setpointTolerance = 1.5; // +/- this value to define range where we don't activate PID
    float stepsPerDegree;
    float degreesPerStep;
    int hall;
    float openLoopReferenceAngle = 0.0;
    float pidReferenceAngle = 0.0;

    ElevatorType type;
    PolynomialCoefficients3 hallAnglePoly3; // Third order polynomial equation fit to valid Hall band for each STBD and PORT elevators: https://docs.google.com/spreadsheets/d/139LkqLD3qmnNt9uvPc4skjA-Np8bwkIl_fo9Cpz7lSg/edit?gid=1133437030#gid=1133437030 and https://docs.google.com/spreadsheets/d/1BEfc8zbWX44ltWZvnlFZUHk_6293ukMDRneSBKh3vh4/edit?gid=1133437030#gid=1133437030 
    HallLimits hallLimits; // min/max valid Hall values we can safely use PID/Hall sensor for
    AngleLimits angleLimits; // degrees, min/max angles we can safely use PID/Hall sensor for
    AccelStepper* stepper;

    double Kp = 50;
    double Ki = 0.5;
    double Kd = 0.01;
    PID* pid;

    void distanceToGo();

    // Nested calibration class. It's tightly coupled to elevator class.
    class Calibrator {
      public:
        Calibrator(Elevator& elevator);
       
        void Calibrator::calibrate();

        void Calibrator::trackWindow();

        void Calibrator::computeMidpoint();

      private:
        Elevator& elevator; // Pointer to outer class

        bool calibrated = false;
        bool trackingComplete = false;
        int loggedLast = false;
        int upperHallLimit = 1000;
        int lowerHallLimit = 800;
        bool upperReady = false;
        bool lowerReady = false;
        float upperAngle = 0.0;
        float lowerAngle = 0.0;
        float midpointAngle = 0.0;
        int hallTolerance = 3;
        float currHallReading = 0;
        float angle = 0.0;
    };

    Calibrator calibrator; // Instance of Calibrator
};

class Elevators
{
  public:
    typedef struct
    {
      Elevator::info port;
      Elevator::info stbd; 
	  } info;

    Elevators();

    // This must be called as frequently as possible by the main loop
    void run();
    
    // 0: steps only; 1: steps+feedback PID
    void setMode(Elevator::ControlMode mode_in);

    // Assign pitch goal
    void assignPitch(float angle);

    // Assign angle goal for a given elevator (degrees)
    void setAngleGoal(ElevatorType elevator, float angle_in);

    // Assign position goal for a given elevator (steps)
    void setPositionGoal(ElevatorType elevator, int position);

    // Return info for each elevator
    Elevators::info getInfo();

    // Run elevators to positions corresponding to hall values
    void runToHalls(int hallStbd, int hallPort);

    // Run calibration routine per elevator
    void calibrate(ElevatorType elevator);

    // Save the reference angle per elevator using the current position of the stepper (steps converted to degrees)
    void setReferenceAngle(ElevatorType elevators);

  private:
    Elevator port;
    Elevator stbd;
};

#endif // ELEVATORS_H