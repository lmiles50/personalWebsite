#include "elevators.h"

// ******** Elevator class ************* //

Elevator::Elevator(uint8_t step, uint8_t dir, uint8_t enable, uint8_t hall_pin, int speed, int max_speed, float acceleration, int sleep_pin, int reset_pin, ElevatorType type)
    : hallPin(hall_pin), angle(0), mode(OPEN_LOOP), input(0), output(0), setpoint(0), speed(speed), maxSpeed(max_speed), 
    acceleration(acceleration), sleepPin(sleep_pin), resetPin(reset_pin),
    pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT),
    calibrator(*this),
    type(type)
{
  stepper = new AccelStepper(AccelStepper::DRIVER, step, dir);
  stepper->setEnablePin(enable);
  stepper->setPinsInverted(false, false, true);
  stepper->disableOutputs();
  stepper->setSpeed(speed);
  stepper->setMaxSpeed(maxSpeed);
  stepper->setAcceleration(acceleration);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(sleepPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  digitalWrite(sleepPin, HIGH);  // Active low
  digitalWrite(resetPin, HIGH);  // Active low

  this->stepsPerDegree = (GEAR_RATIO * STEPS_PER_REV / MICROSTEPPING) / 360.0; 
  this->degreesPerStep = 1.0 / this->stepsPerDegree;
  this->hallScaleDegPerBit = DEGREES_PER_REVOLUTION / A2D_SENSOR_RANGE;

  // Select polynomial coefficients and angle/limits based on ElevatorType
  if (type == STBD) {
    this->hallAnglePoly3 = {5.897066800086291e+3, -19.743454492547105, 0.022479264442493, -8.380382751528903e-6};
    this->hallLimits = {775, 1016};
    this->angleLimits = {-30, 30};
  } else if (type == PORT) {
    this->hallAnglePoly3 = {5.144977550166169e+3, -17.725381248910047, 0.020202405494407, -7.534274024498004e-6};
    this->hallLimits = {769, 1016};
    this->angleLimits = {-30, 30};
  }

  // Here:
  // - input is the actual Hall readout angle
  // - setpoint is the desired Hall readout target
  // - output is the position which the elevator should move to //old: is the speed at which the elevator should move
  pid = new PID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
  Serial.begin(115200);

  pid->SetMode(AUTOMATIC);
  // pid->SetOutputLimits(-180, 180); degrees, position-based control
  pid->SetOutputLimits(ELEVATOR_MIN_SPEED, ELEVATOR_MAX_SPEED); // speed-based control
}

void Elevator::run()
{
  // Read the hall sensor and convert to angle
  this->hall = analogRead(hallPin);
  this->input = this->calculatePolynomial3(this->hall, this->hallAnglePoly3); // third order poly fit, degrees
  this->input -= this->pidReferenceAngle; // keep relative to hall zero point, see fit to average hall curve at: https://docs.google.com/spreadsheets/d/139LkqLD3qmnNt9uvPc4skjA-Np8bwkIl_fo9Cpz7lSg/edit?gid=1133437030#gid=1133437030 or https://docs.google.com/spreadsheets/d/1BEfc8zbWX44ltWZvnlFZUHk_6293ukMDRneSBKh3vh4/edit?gid=1133437030#gid=1133437030

  // Run the PID-loop or open-loop
  if (mode == Elevator::ControlMode::OPEN_LOOP)
  {
    stepper->runSpeedToPosition();
    this->distanceToGo();
  }
  else if (mode == Elevator::ControlMode::HALL_PID)
  {
    if (this->type == ElevatorType::PORT) {
      Serial.print("setpoint: ");
      Serial.println(this->setpoint);
      Serial.print("hall: ");
      Serial.println(this->hall);
      Serial.print("input: ");
      Serial.println(this->input);
      Serial.print("openLoopReference angle: ");
      Serial.println(this->openLoopReferenceAngle);
      Serial.print("pidReference angle: ");
      Serial.println(this->pidReferenceAngle);
    }

    // First check if we are venturing outside the angle range cooresponding to valid Hall values. If we are, then we use open-loop control.
    // Otherwise, two cases ensue: 1) we are outside of the valid Hall range but wish to return somewhere to it, 2) we are within the valid Hall range and wish to stay in it
    if (this->setpoint <= this->angleLimits.min || this->setpoint >= this->angleLimits.max) {
      Serial.println("move to invalid Hall band");
      stepper->setSpeed(this->speed);
      stepper->runSpeedToPosition();
      this->distanceToGo();
    } else if (this->hall <= this->hallLimits.min) { // we are below valid Hall range, step until we are confident we are in the valid Hall range
      Serial.println("move from invalid Hall band: lower");
      stepper->moveTo(stepper->currentPosition() + 1);
      stepper->setSpeed(this->speed);
      stepper->runSpeedToPosition(); 
      this->distanceToGo();
    } else if (this->hall >= this->hallLimits.max) { // we are above valid Hall range, step until we are confident we are in the valid Hall range
      Serial.println("move from invalid Hall band: upper");
      stepper->moveTo(stepper->currentPosition() - 1);
      stepper->setSpeed(this->speed);
      stepper->runSpeedToPosition(); 
      this->distanceToGo();
    } else {
      Serial.println("valid Hall band");
      pid->Compute();  // Calculate PID control signal

      if (abs(this->input - this->setpoint) > this->setpointTolerance) {
        // speed-based control
        stepper->setSpeed(this->output);  // Use PID output as speed command
        stepper->runSpeed();  // Run the stepper at the calculated speed

        // For HALL_PID mode, keep the motor outputs enabled
        digitalWrite(LED_BUILTIN, LOW);
        stepper->enableOutputs();
      }
    }
  }
}

float Elevator::calculatePolynomial3(float x, PolynomialCoefficients3 poly3) {
  return poly3.d * x * x * x + poly3.c * x * x + poly3.b * x + poly3.a;
}

void Elevator::runToHall(int hall) {
  this->hall = analogRead(hallPin);

  while (this->hall != hall) {
    //TODO: verify the +/- are in the right cases
    if (hall > this->hall) {
      stepper->moveTo(stepper->currentPosition() + 1);
    } else {
      stepper->moveTo(stepper->currentPosition() - 1);
    }
    
    stepper->setSpeed(this->speed);
    stepper->runSpeedToPosition(); 
    this->distanceToGo();

    this->hall = analogRead(hallPin);
  }
}

void Elevator::distanceToGo() 
{
  if (stepper->distanceToGo() == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    stepper->disableOutputs();
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    stepper->enableOutputs();
  }
}

void Elevator::setDynamics(double speed, double max_speed, double acceleration)
{
  this->speed = speed;
  this->maxSpeed = max_speed;
  this->acceleration = acceleration;

  stepper->setSpeed(this->speed); // needs to be called after moveTo for constant speed: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#ace236ede35f87c63d18da25810ec9736
  stepper->setMaxSpeed(this->maxSpeed);
  stepper->setAcceleration(this->acceleration);
}

void Elevator::setAngleGoal(double angle_in)
{
  this->angle = angle_in;

  if (mode == Elevator::ControlMode::OPEN_LOOP)
  {
    int steps = (int) ((angle_in + this->openLoopReferenceAngle) * this->stepsPerDegree); // convert angle to steps, use reference angle
    stepper->moveTo(steps);
    stepper->setSpeed(this->speed); // needs to be called after moveTo for constant speed: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#ace236ede35f87c63d18da25810ec9736
  }
  else if (mode == Elevator::ControlMode::HALL_PID)
  {
    this->setpoint = this->angle;
  }
}

void Elevator::setPositionGoal(int position)
{
  this->angle = position / this->stepsPerDegree; // convert steps to angle to keep things in angles internally
  position = position + (this->openLoopReferenceAngle * this->stepsPerDegree); // use reference angle
  stepper->moveTo(position);
  stepper->setSpeed(this->speed);
}

void Elevator::setMode(Elevator::ControlMode mode_in)
{
  mode = mode_in;
}

Elevator::info Elevator::getInfo()
{
  Elevator::info info;

  info.angle = this->angle;
  info.mode = this->mode;
  info.hallPin = this->hallPin;
  info.resetPin = this-> resetPin;
  info.sleepPin = this->sleepPin;
  info.hallScaleDegPerBit = this->hallScaleDegPerBit;
  info.speed = this->speed;
  info.maxSpeed = this->maxSpeed;
  info.acceleration = this->acceleration;
  info.input = this->input;
  info.output = this->output;
  info.setpoint = this->setpoint;
  info.stepsPerDegree = this->stepsPerDegree;
  info.hall = this->hall;
  info.distanceToGo = this->stepper->distanceToGo();
  info.currentPosition = this->stepper->currentPosition();
  info.openLoopReferenceAngle = this->openLoopReferenceAngle;
  info.pidReferenceAngle = this->pidReferenceAngle;

  return info;
}

void Elevator::calibrate() {
  calibrator.calibrate();  // Simply delegate to the Calibrator's calibrate method
}

void Elevator::setReferenceAngle() {
  this->openLoopReferenceAngle = this->stepper->currentPosition() * this->degreesPerStep;
  this->pidReferenceAngle = this->calculatePolynomial3(this->getInfo().hall, this->hallAnglePoly3);

  String elevatorName = (this->type == ElevatorType::PORT) ? "PORT" : "STBD"; 
  Serial.print(elevatorName);
  Serial.print("::setReferenceAngle::open loop zero point angle: ");
  Serial.println(this->openLoopReferenceAngle);
  Serial.print(elevatorName);
  Serial.print("::setReferenceAngle::pid zero point angle: ");
  Serial.println(this->pidReferenceAngle);
}

// ******** Implementation of Calibrator nested class methods ************* //

Elevator::Calibrator::Calibrator(Elevator& elevator) : elevator(elevator) {
    // Constructor implementation
}

void Elevator::Calibrator::calibrate() {
  // Reset state to allow calibration to be done multiple times
  this->calibrated = false;
  this->trackingComplete = false;
  this->loggedLast = false;
  this->upperReady = false;
  this->lowerReady = false;
  this->angle = 0.0;
  this->elevator.openLoopReferenceAngle = 0.0;
  this->elevator.pidReferenceAngle = 0.0;
  this->elevator.stepper->setCurrentPosition(0); // Reset stepper's position counter. NOTE: resetting the hardware driver's reset pin doesn't do this

  String elevatorName = (this->elevator.type == ElevatorType::PORT) ? "PORT" : "STBD"; 
  this->elevator.setAngleGoal(420);

  while (!this->calibrated) {
    this->elevator.run();

    // If the elevator is still moving, continue processing
    if (this->trackingComplete == false && this->elevator.getInfo().distanceToGo > 0) { // find zero point i.e. midpoint of most suitable angular gap
      trackWindow();
    } else if (this->trackingComplete == false) { // drive to zero point
      this->trackingComplete = true;
      this->elevator.setAngleGoal(this->midpointAngle);// - this->elevator.openLoopReferenceAngle);
    } else if (this->trackingComplete == true && this->elevator.getInfo().distanceToGo == 0 && this->loggedLast == false) { // log final hall value
      this->loggedLast = true;
      this->elevator.pidReferenceAngle = this->elevator.calculatePolynomial3(this->elevator.getInfo().hall, this->elevator.hallAnglePoly3);
      Serial.print(elevatorName);
      Serial.print("::calibration::final hall: ");
      Serial.println(this->elevator.getInfo().hall);
    } else if (this->trackingComplete == true && this->elevator.getInfo().distanceToGo == 0 && this->loggedLast == true) { // completed
      this->calibrated = true;
      this->elevator.openLoopReferenceAngle = this->midpointAngle;
      Serial.print(elevatorName);
      Serial.print("::calibration::zero point angle: ");
      Serial.println(this->elevator.openLoopReferenceAngle);
      Serial.print(elevatorName);
      Serial.print("::calibration::pid reference angle: ");
      Serial.println(this->elevator.pidReferenceAngle);
    }
    
    delay(2); // we step too fast and increment angle too much if we don't delay a bit
  }
}

void Elevator::Calibrator::trackWindow() {
  this->currHallReading = analogRead(this->elevator.hallPin);

  if (this->currHallReading <= (this->upperHallLimit + this->hallTolerance) && this->currHallReading >= (this->upperHallLimit - this->hallTolerance)) {
    this->upperAngle = this->angle;// + this->elevator.openLoopReferenceAngle; // If this is not the first calibration, then we need to account for the current reference angle
    this->upperReady = true;
  } else if (this->currHallReading <= (this->lowerHallLimit + this->hallTolerance) && this->currHallReading >= (this->lowerHallLimit - this->hallTolerance)) {
    this->lowerAngle = this->angle;// + this->elevator.openLoopReferenceAngle; // If this is not the first calibration, then we need to account for the current reference angle;
    this->lowerReady = true;
  } else if (this->currHallReading > this->upperHallLimit) {
    this->upperReady = false;
    this->lowerReady = false;
  } else if (this->currHallReading < this->lowerHallLimit) {
    this->upperReady = false;
    this->lowerReady = false;
  }

  if (this->upperReady && this->lowerReady) {
    this->computeMidpoint();
  }

  this->angle += this->elevator.degreesPerStep; // Update the current angle
}

void Elevator::Calibrator::computeMidpoint() {
  float width = abs(this->upperAngle - this->lowerAngle);

  if (width > 20.0 && width < 60.0) {
    this->midpointAngle = ((this->upperAngle + this->lowerAngle) / 2.0);
  }
}

// ******** Elevators class ************* //

Elevators::Elevators()
    : port(STEPPER_PORT_STEP_PIN, STEPPER_PORT_DIR_PIN, STEPPER_PORT_ENABLE_PIN, HALL_A2D_PORT_PIN, PORT_ELEVATOR_SPEED, PORT_ELEVATOR_SPEED, PORT_ELEVATOR_ACCELERATION, STEPPER_PORT_SLEEP_PIN, 
    STEPPER_PORT_RESET_PIN, ElevatorType::PORT),
      stbd(STEPPER_STBD_STEP_PIN, STEPPER_STBD_DIR_PIN, STEPPER_STBD_ENABLE_PIN, HALL_A2D_STBD_PIN, STBD_ELEVATOR_SPEED, STBD_ELEVATOR_SPEED, STBD_ELEVATOR_ACCELERATION, STEPPER_STBD_SLEEP_PIN, 
      STEPPER_STBD_RESET_PIN, ElevatorType::STBD)
{
}

void Elevators::run()
{
  port.run();
  stbd.run();
}

void Elevators::setMode(Elevator::ControlMode mode_in)
{
  this->port.setMode(mode_in);
  this->stbd.setMode(mode_in);
}

void Elevators::assignPitch(float angle) 
{
  this->port.setAngleGoal(angle); 
  this->stbd.setAngleGoal(angle);
}

void Elevators::setAngleGoal(ElevatorType elevator, float angle_in)
{
  switch (elevator) {
    case ElevatorType::PORT:
      this->port.setAngleGoal(angle_in);
      break;
    case ElevatorType::STBD:
      this->stbd.setAngleGoal(angle_in);
      break;
  }
}

void Elevators::setPositionGoal(ElevatorType elevator, int position)
{
  switch (elevator) {
    case ElevatorType::PORT:
      this->port.setPositionGoal(position);
      break;
    case ElevatorType::STBD:
      this->stbd.setPositionGoal(position);
      break;
  }
}

Elevators::info Elevators::getInfo()
{
  Elevators::info info;

  info.port = this->port.getInfo();
  info.stbd = this->stbd.getInfo();

  return info;
}

void Elevators::runToHalls(int hallStbd, int hallPort) {
  this->stbd.runToHall(hallStbd);
  this->port.runToHall(hallPort);
}

void Elevators::calibrate(ElevatorType elevator) {
  if (elevator == ElevatorType::PORT) {
    this->port.calibrate();
  } else if (elevator == ElevatorType::STBD) {
    this->stbd.calibrate();
  } else {
    Serial.print("calibrate::Unknown elevator type: ");
    Serial.println(elevator);
  }
}

void Elevators::setReferenceAngle(ElevatorType elevator) {
  if (elevator == ElevatorType::PORT) {
    this->port.setReferenceAngle();
  } else if (elevator == ElevatorType::STBD) {
    this->stbd.setReferenceAngle();
  } else {
    Serial.print("setReferenceAngle::Unknown elevator type: ");
    Serial.println(elevator);
  }
}