#ifndef NAV_H
#define NAV_H

#include "epic_daug_defines.h"
#include "elevators.h"
#include "vesc_propeller.h"
#include <PID_v1.h>

class Nav
{
  public:
    typedef struct
    {
      float data; // actual data
      unsigned long ts; // millis timestamp of the data
    } StampedData;

    Nav(Elevators& elevators, VescPropeller& propeller);

    void assignSpeed(float data_in, unsigned long ts_in) {speed.data = data_in; speed.ts = ts_in;};
    void assignDepth(float data_in, unsigned long ts_in) {depth.data = data_in; depth.ts = ts_in;};
    void assignHeading(float data_in, unsigned long ts_in) {heading.data = data_in; heading.ts = ts_in;};
    void assignPitch(float data_in, unsigned long ts_in) {pitch.data = data_in; pitch.ts = ts_in;};
    void assignRoll(float data_in, unsigned long ts_in) {roll.data = data_in; roll.ts = ts_in;};

    float get_speed() {return speed.data;};
    float get_depth() {return depth.data;};
    float get_heading() {return heading.data;};
    float get_pitch() {return pitch.data;};
    float get_roll() {return roll.data;};

    void setThrusterCmdType(int type) { thruster_cmd_type = type; }

    void run(const VescPropeller::imuPackage &imu_data);

  private:
    StampedData speed; // Target speed
    StampedData current_speed; // Actual speed
    StampedData depth;
    StampedData heading;
    StampedData pitch;
    StampedData roll;

    float thruster_cmd = 0;
    unsigned long last_update = 0;
    const unsigned long control_interval = 100; // Update interval in ms
    int thruster_cmd_type = THRUSTER_CMD_AMPS;

    // Speed PID parameters for prop
    double sKp = 1.0, sKi = 0.5, sKd = 0.1;
    double sInput, sOutput, sSetpoint;
    PID sPid;

    // Pitch PID parameters for elevators
    double pKp = 1.0, pKi = 0.5, pKd = 0.1;
    double pInput, pOutput, pSetpoint;
    PID pPid;

    Elevators* elevators;
    VescPropeller* prop;

    // Pitch limits (degrees)
    float pPitchLimit = 45; // degrees
    float nPitchLimit = -45; // degrees
    float pitchTolerance = 3.0; // degrees

    void updateCurrentSpeedFromIMU();
};

#endif // NAV_H