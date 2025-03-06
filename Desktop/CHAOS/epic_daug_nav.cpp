#include "epic_daug_nav.h"

Nav::Nav(Elevators& elevators, VescPropeller& propeller)
  : sPid(&sInput, &sOutput, &sSetpoint, sKp, sKi, sKd, DIRECT),
    pPid(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT),
    elevators(&elevators),
    prop(&propeller)
{
    sPid.SetMode(AUTOMATIC);
    pPid.SetMode(AUTOMATIC);

    pPid.SetOutputLimits(-180, 180);

    Serial.begin(115200);
}

void Nav::run(const VescPropeller::imuPackage &imu_data) {
    /**** Speed ****/

    // Update the current speed from IMU data
    updateCurrentSpeedFromIMU();

    // Set the setpoint for the speed PID controller
    sSetpoint = speed.data;
    sInput = current_speed.data;

    // Compute the speed PID output
    sPid.Compute();

    // Update the thruster command based on PID output
    thruster_cmd = sOutput;

    // Apply the thruster command based on the thruster_cmd_type
    switch (thruster_cmd_type) {
        case THRUSTER_CMD_RPM:
            prop->setRPM(thruster_cmd);
            break;
        case THRUSTER_CMD_AMPS:
            prop->setCurrent(thruster_cmd);
            break;
        case THRUSTER_CMD_DUTY:
            prop->setDuty(thruster_cmd);
            break;
    }

    /**** Pitch ****/

    // Use bang-bang controller when pitch out of limits
    float pitch_degrees = imu_data.pitch * (180.0 / PI); // IMU pitch data in radians
    // Serial.println(pitch_degrees);
    if (pitch_degrees >= pPitchLimit) {
      Serial.println("here");
      elevators->setMode(Elevator::ControlMode::OPEN_LOOP);
      elevators->assignPitch(nPitchLimit);
    } else if (pitch_degrees <= nPitchLimit) {
      Serial.println("there");
      elevators->setMode(Elevator::ControlMode::OPEN_LOOP);
      elevators->assignPitch(pPitchLimit);
    } else if (abs(pitch_degrees - this->pitch.data) > pitchTolerance) { // Use PID when within limits
      // Set the setpoint for the pitch PID controller
      pSetpoint = this->pitch.data;
      pInput = pitch_degrees;

      // Debugging prints
      Serial.print("pInput: ");
      Serial.println(pInput);
      Serial.print("pSetpoint: ");
      Serial.println(pSetpoint);

      // Compute the pitch PID output
      pPid.Compute();

      // float pOutput_degrees = pOutput * (180.0 / PI);

      Serial.print("pOutput: ");
      Serial.println(pOutput);

      // Update the elevator command based on PID output
      // elevators->setMode(Elevator::ControlMode::OPEN_LOOP); // TODO remove this
      elevators->setMode(Elevator::ControlMode::HALL_PID);
      elevators->assignPitch(pOutput);
    }
 
    // Run the elevators
    elevators->run();
}

// Very rough velocity calculation based only on IMU
// Will likely get replaced by DVL
void Nav::updateCurrentSpeedFromIMU() {
    static unsigned long last_time = millis();
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // Convert to seconds

    VescPropeller::imuPackage imu_data = prop->getTheIMU();

    // Compute acceleration magnitude (assuming flat movement and ignoring gravity for simplicity)
    float accel_x = imu_data.accel_x;
    float accel_y = imu_data.accel_y;
    float accel_z = imu_data.accel_z;

    // Integrate acceleration to get speed (simple numerical integration)
    static float velocity_x = 0;
    static float velocity_y = 0;
    static float velocity_z = 0;

    velocity_x += accel_x * dt;
    velocity_y += accel_y * dt;
    velocity_z += accel_z * dt;

    // Compute speed magnitude (approximating for now)
    float calculated_speed = sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z);

    current_speed.data = calculated_speed;
    current_speed.ts = millis();

    last_time = current_time; // Update last_time after the computation
}