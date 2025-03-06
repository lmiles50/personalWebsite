#include "epic_daug_includes.h"
#include "epic_daug_defines.h"

Elevators elevators;
VescPropeller prop;
Nav navigation(elevators, prop);

// Buffers for commands from backseat
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// Counter to decrease the message spams
unsigned long previousMillis_1Hz = 0;
unsigned long interval_1Hz = 1000;
unsigned long previousMillis_10Hz = 0;
unsigned long interval_10Hz = 100;

bool overridePots = false;  // If true, don't use the test potentiometers for elevators and thruster commands but use backseat commands

// Controls
int backseat_cmd_type = BackseatCommandTypes::CONTROL;
float thruster_cmd = 0;
int thruster_cmd_type = 0;
int stepper_port_cmd = 0;
int stepper_stbd_cmd = 0;

// Elevators
float steps_per_degree;
Elevators::info elevatorsInfo;

// VESC IMU data
VescPropeller::imuPackage imu_data;

// VESC Prop data
VescPropeller::dataPackage prop_data;

void setup() {
  steps_per_degree = (GEAR_RATIO * STEPS_PER_REV / MICROSTEPPING) / DEGREES_PER_REVOLUTION;
  pinMode(LED_BUILTIN, OUTPUT);

  // This is the USB serial connection
  Serial.begin(115200);

  // Vesc port
  Serial8.begin(115200);
  while (!Serial8) { ; }  // Wait for serial to spin up
  //UART.setSerialPort(&Serial8);
  prop.setSerialPort(&Serial8);

  // Front Seat Computer (Persistor) TODO: Add in for real system
  Serial1.begin(115200);
  while (!Serial1) { ; }  // Wait for serial to spin up
  //UART.setSerialPort(&Serial1);

  // Only need to do this once
  prop.requestFWVersion();

  prop.setRPM(762.0);
}

void loop() {
  // these run() commands should be ran as fast and regularly as possible
  if (backseat_cmd_type == BackseatCommandTypes::CONTROL || backseat_cmd_type == BackseatCommandTypes::RESET) {
    elevators.run();
  } else if (backseat_cmd_type == BackseatCommandTypes::NAV) {
    navigation.run(imu_data);
  }

  // ******** 10Hz Mid frequency loop ************* //

  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis_10Hz) >= interval_10Hz) {
    previousMillis_10Hz = currentMillis;

#if USE_THRUSTER
    if (backseat_cmd_type == BackseatCommandTypes::CONTROL) {
      int val_thruster = analogRead(POTENTIOMETER_THRUSTER) / 10;  // decrease resolution

      // THRUSTER RPM
      if (thruster_cmd_type == THRUSTER_CMD_RPM) {
        if (!overridePots)
          thruster_cmd = map(val_thruster, 0, 102, -8000, 8000);
        //UART.setRPM(thruster_cmd);
        prop.setRPM(thruster_cmd);
      }
      // THRUSTER DUTY CYCLE
      if (thruster_cmd_type == THRUSTER_CMD_DUTY) {
        if (!overridePots)
          thruster_cmd = map(val_thruster, 0, 102, -100, 100) / 1000.0;
        //UART.setDuty(thruster_cmd);
        prop.setDuty(thruster_cmd);
      }
      // THRUSTER CURRENT
      if (thruster_cmd_type == THRUSTER_CMD_AMPS) {
        if (!overridePots)
          thruster_cmd = map(val_thruster, 0, 102, -100, 100) / 100.0;
        //UART.setCurrent(thruster_cmd);
        prop.setCurrent(thruster_cmd);
      }
    }

#endif
  }

  // ******** 1Hz Low frequency loop ************* //

  if ((currentMillis - previousMillis_1Hz) >= interval_1Hz) {
    previousMillis_1Hz = currentMillis;

    StaticJsonDocument<1024> doc;

    elevatorsInfo = elevators.getInfo();
    
    doc["type"] = "ELEVATORS";

    // port elevator
    doc["port_angle"] = elevatorsInfo.port.angle;
    doc["port_mode"] = (elevatorsInfo.port.mode == Elevator::OPEN_LOOP) ? "OPEN_LOOP" : "HALL_PID";
    doc["port_hallPin"] = elevatorsInfo.port.hallPin;
    doc["port_resetPin"] = elevatorsInfo.port.resetPin;
    doc["port_sleepPin"] = elevatorsInfo.port.sleepPin;
    doc["port_hallScaleDegPerBit"] = elevatorsInfo.port.hallScaleDegPerBit;
    doc["port_speed"] = elevatorsInfo.port.speed;
    doc["port_maxSpeed"] = elevatorsInfo.port.maxSpeed;
    doc["port_acceleration"] = elevatorsInfo.port.acceleration;
    doc["port_pid_input"] = elevatorsInfo.port.input;
    doc["port_pid_output"] = elevatorsInfo.port.output;
    doc["port_setpoint"] = elevatorsInfo.port.setpoint;
    doc["port_stepsPerDegree"] = elevatorsInfo.port.stepsPerDegree;
    doc["port_hall"] = elevatorsInfo.port.hall;
    doc["port_distanceToGo"] = elevatorsInfo.port.distanceToGo;
    doc["port_currentPosition"] = elevatorsInfo.port.currentPosition;
    doc["port_openLoopReferenceAngle"] = elevatorsInfo.port.openLoopReferenceAngle;
    doc["port_pidReferenceAngle"] = elevatorsInfo.port.pidReferenceAngle;
    
    // stbd elevator
    doc["stbd_angle"] = elevatorsInfo.stbd.angle;
    doc["stbd_mode"] = (elevatorsInfo.stbd.mode == Elevator::OPEN_LOOP) ? "OPEN_LOOP" : "HALL_PID";
    doc["stbd_hallPin"] = elevatorsInfo.stbd.hallPin;
    doc["stbd_resetPin"] = elevatorsInfo.stbd.resetPin;
    doc["stbd_sleepPin"] = elevatorsInfo.stbd.sleepPin;
    doc["stbd_hallScaleDegPerBit"] = elevatorsInfo.stbd.hallScaleDegPerBit;
    doc["stbd_speed"] = elevatorsInfo.stbd.speed;
    doc["stbd_maxSpeed"] = elevatorsInfo.stbd.maxSpeed;
    doc["stbd_acceleration"] = elevatorsInfo.stbd.acceleration;
    doc["stbd_pid_input"] = elevatorsInfo.stbd.input;
    doc["stbd_pid_output"] = elevatorsInfo.stbd.output;
    doc["stbd_setpoint"] = elevatorsInfo.stbd.setpoint;
    doc["stbd_stepsPerDegree"] = elevatorsInfo.stbd.stepsPerDegree;
    doc["stbd_hall"] = elevatorsInfo.stbd.hall;
    doc["stbd_distanceToGo"] = elevatorsInfo.stbd.distanceToGo;
    doc["stbd_currentPosition"] = elevatorsInfo.stbd.currentPosition;
    doc["stbd_openLoopReferenceAngle"] = elevatorsInfo.stbd.openLoopReferenceAngle;
    doc["stbd_pidReferenceAngle"] = elevatorsInfo.stbd.pidReferenceAngle;

    doc["millis"] = currentMillis;

    // serializeJson(doc, Serial);
    serializeJsonPretty(doc, Serial);
    Serial.println();

    prop.requestVescValues();
    prop.requestImuData();
  }

  // ******** READ BYTES FROM SERIAL PORTS ************* //

  // process inputs/commands
  readSerial();
  if (stringComplete) {
    // type: NAV; contains: SPEED, DEPTH, HEADING, PITCH, ROLL
    // type: CONTROL_LEVEL, contans: integer enum specifying active API level
    // type: CONTROL_1, contains: PORT_ELEVATOR_ANGLE, STBD_ELEVATOR_ANGLE, THRUSTER_RPM
    // type: CONTROL_2, contains: GOAL_DEPTH, GOAL_SPEED
    // type: RESET, enables potentionmeter for thruster control, sets thruster_cmd_type to THRUSTER_CMD_RPM, sets thruster_cmd to 0, moves the steppers to their 0 positions
    // type: SET_ELEVATOR_REFERENCE_ANGLE, sets the reference angle (per elevator) to be the current position of the elevator
    // type: CALIBRATE_ELEVATOR, finds a suitable reference angle (per elevator)
    Serial.println("inputString");
    Serial.println(inputString);

    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, inputString);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    } else {
      backseat_cmd_type = doc["type"];

      if (backseat_cmd_type == BackseatCommandTypes::NAV) {
        elevators.setMode(Elevator::ControlMode::HALL_PID);

        unsigned long now = millis();
        navigation.assignDepth(doc["depth"], now);
        navigation.assignSpeed(doc["speed"], now);
        navigation.assignHeading(doc["heading"], now);
        navigation.assignPitch(doc["pitch"], now);
        navigation.assignRoll(doc["roll"], now);
      } else if (backseat_cmd_type == BackseatCommandTypes::CONTROL) {
        elevators.setMode(Elevator::ControlMode::OPEN_LOOP);

        //int level = doc["level"];
        in level = 2

        if (level == 1) { // steppers are commanded steps to move through
          stepper_port_cmd = doc["step_port"];
          elevators.setPositionGoal(ElevatorType::PORT, stepper_port_cmd);
          stepper_stbd_cmd = doc["step_stbd"];
          elevators.setPositionGoal(ElevatorType::STBD, stepper_stbd_cmd);
          thruster_cmd = doc["thruster_cmd"];
          thruster_cmd_type = doc["thruster_cmd_type"];
          overridePots = true;
        }
        if (level == 2) { // steppers are commanded angles to move to
          stepper_port_cmd = (int)((float)doc["step_port"] * steps_per_degree);
          elevators.setAngleGoal(ElevatorType::PORT, (float)doc["step_port"]);
          stepper_stbd_cmd = (int)((float)doc["step_stbd"] * steps_per_degree);
          elevators.setAngleGoal(ElevatorType::STBD, (float)doc["step_stbd"]);
          thruster_cmd = doc["thruster_cmd"];
          thruster_cmd_type = doc["thruster_cmd_type"];
          overridePots = true;
        } 
      } else if (backseat_cmd_type == BackseatCommandTypes::RESET) {
        elevators.setMode(Elevator::ControlMode::OPEN_LOOP);

        overridePots = false;
        thruster_cmd = 0;
        thruster_cmd_type = 0;
        stepper_port_cmd = 0;
        elevators.setPositionGoal(ElevatorType::PORT, stepper_port_cmd);
        stepper_stbd_cmd = 0;
        elevators.setPositionGoal(ElevatorType::STBD, stepper_stbd_cmd);
      } else if (backseat_cmd_type == BackseatCommandTypes::SET_ELEVATOR_REFERENCE_ANGLE) {
        elevators.setMode(Elevator::ControlMode::OPEN_LOOP);

        //int level = doc["level"];
        int level = 2

        if (level == 1) { // save PORT
          elevators.setReferenceAngle(ElevatorType::PORT);
        }
        else if (level == 2) { // save STBD
          elevators.setReferenceAngle(ElevatorType::STBD);
        }
        else if (level == 3) { // save both
          elevators.setReferenceAngle(ElevatorType::PORT);
          elevators.setReferenceAngle(ElevatorType::STBD);
        }
      } else if (backseat_cmd_type == BackseatCommandTypes::CALIBRATE_ELEVATOR) {
        elevators.setMode(Elevator::ControlMode::OPEN_LOOP);

        int level = doc["level"];

        if (level == 1) { // calibrate PORT
          elevators.calibrate(ElevatorType::PORT);
        }
        else if (level == 2) { // calibrate STBD
          elevators.calibrate(ElevatorType::STBD);
        }
        else if (level == 3) { // calibrate both
          elevators.calibrate(ElevatorType::PORT);
          elevators.calibrate(ElevatorType::STBD);
        }
      }
    }
    inputString = "";
    stringComplete = false;
  }

  // Read Vesc serial. Returns bool if we have a full packet
  bool packet_rxd = readSerial8();
  if (packet_rxd) {
    VescPropeller::FWversionPackage fw_version = prop.getTheFW();
    prop_data = prop.getTheData();
    imu_data = prop.getTheIMU();

    StaticJsonDocument<1024> doc;

    // Propulsion
    doc["type"] = "PROPULSION";
    doc["cmd"] = thruster_cmd;
    doc["rpm"] = prop_data.rpm;
    doc["Vin"] = prop_data.inpVoltage;
    doc["Ahrs"] = prop_data.ampHours;
    //Lmiles Edit Start 
    doc["system_current_draw"] = prop_data.avgInputCurrent;
    doc["motor_current_draw"]= prop_data.avgMotorCurrent;
    doc["duty_cycle"] = prop_data.dutyCycleNow;
    //Lmiles Edit End  
    doc["tach"] = prop_data.tachometer;
    doc["Temp"] = prop_data.tempMosfet;
    // IMU
    doc["roll"] = imu_data.roll;
    doc["pitch"] = imu_data.pitch;
    doc["yaw"] = imu_data.yaw;
    doc["accel_x"] = imu_data.accel_x;
    doc["accel_y"] = imu_data.accel_y;
    doc["accel_z"] = imu_data.accel_z;
    doc["gyro_x"] = imu_data.gyro_x;
    doc["gyro_y"] = imu_data.gyro_y;
    doc["gyro_z"] = imu_data.gyro_z;
    doc["mag_x"] = imu_data.mag_x;
    doc["mag_y"] = imu_data.mag_y;
    doc["mag_z"] = imu_data.mag_z;
    doc["quat_w"] = imu_data.quat_w;
    doc["quat_x"] = imu_data.quat_x;
    doc["quat_y"] = imu_data.quat_y;
    doc["quat_z"] = imu_data.quat_z;
    // FW
    doc["majorFW"] = fw_version.major;
    doc["minorFW"] = fw_version.minor;

    // serializeJson(doc, Serial);
    serializeJsonPretty(doc, Serial);
    Serial.println();
  }

  // Example loop-back commands for Serial1 to emulate Frontseat
  // Command Speed
  // Serial1.write(0xE1);
  // Serial1.write(0x50);
  // Request Current
  // Serial1.write(0x85);
  //Request RPM
  // Serial1.write(0x8F);

  // Read Front Seat Serial
  readSerial1();
}

void readSerial() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '}') {  // Json final delimiter
      stringComplete = true;
    }
  }
}

bool readSerial8() {
  while (Serial8.available()) {
    // get the new byte:
    uint8_t inByte = (uint8_t)Serial8.read();
    // Add it to the buffer of the VescPropeller class and return true if a full packet has been received
    bool full = prop.feed(inByte);
    if (full) {
      bool ok = prop.unpackPayload();
      if (ok) {
        prop.processReadPacket(prop.getPayload());
        return true;
      }
    }
  }
  return false;
}

void readSerial1() {
  // TODO: remove loop-back on Serial1 and integrate with real front seat

  int cmdByte = 0;
  int dataByte = 0;

  if (Serial1.available() > 0) {
    // read the incoming byte:
    cmdByte = Serial1.read();
    
    if (cmdByte == 0xe1) {
      Serial.print("Rx Frontseat command: 0x");
      Serial.println(cmdByte, HEX);

      // backseat_cmd_type = BackseatCommandTypes::FrontSeat;
      //there is a speed command, get the data
      dataByte = Serial1.read();
      Serial.print("Data byte: 0x");
      Serial.println(dataByte, HEX); // Print the byte in hexadecimal format
      //set the speed
      if(dataByte > 0 && dataByte < 128) {
        overridePots = true;
        thruster_cmd = map(dataByte, 1, 127, -8000, 8000);
        Serial.println(thruster_cmd);
        //UART.setRPM(thruster_cmd);
        // prop.setRPM(thruster_cmd);
      } else {
        //stop thruster if not valid speed or is zero
        thruster_cmd = 0;
        // prop.setRPM(0);
      }
    } else if (cmdByte == 0x8f) {
      Serial.print("Rx Frontseat command: 0x");
      Serial.println(cmdByte, HEX);

      // backseat_cmd_type = BackseatCommandTypes::FrontSeat;
      //read the current
      float ampHours = prop_data.ampHours; // TODO: is this the right way to report current?
      Serial.println(ampHours, 8);
      Serial1.write((byte*)&ampHours, sizeof(ampHours));
    } else if (cmdByte == 0x85) {
      Serial.print("Rx Frontseat command: 0x");
      Serial.println(cmdByte, HEX);

      // backseat_cmd_type = BackseatCommandTypes::FrontSeat;
      //read the spead
      float rpm = prop_data.rpm;
      Serial.println(rpm);
      Serial1.write((byte*)&rpm, sizeof(rpm)); // Send the float as bytes
    } 
  }
}