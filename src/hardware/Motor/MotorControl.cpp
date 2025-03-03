/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli   R&D Team VT CRO
// filename: MotorControl.cpp
// Last Modified: 01/14/2024
// Description:  This where the function defintions for the
//               Motor Controller are kept. Continously be updated
//
// Modifications: Changed it from handleing all the motors to just one
//                so that we can take advantage of c++ objects and threads
//                with RTOS
/////////////////////////////////////////////////////////////
#include <hardware/Motor/MotorControl.h>

// Notes:
//  Have a function that allows the encoder to listen to state of motor
//  of the encoder

// Make the functionality for the PID loop - complete?

// Motor Control Constructor
// Assigns values to the pin variables
MotorControl::MotorControl(int in1, int in2) {

  Assignments.in1 = in1;
  Assignments.in2 = in2;
  goal_velocity = 0;
  current_velocity = 0;
  goal_velocity = 0;
  pidMode = false;
  Motor_pin_init();
  last_pidTime = millis();
}

void MotorControl::Motor_setPIDParams(float P, float I, float D) {
  pid = pidInit(P, I, D, 0, 0, 0, 0);
}

// Motor Speed sets the speed of the motors.
// Values go from 0 - 255 for analogWrite.
//  I want to support negative values later
//  to signify reverse.
void MotorControl::Motor_setGoalSpeed(float newSpeed) {
  goal_velocity = newSpeed;
}

void MotorControl::Motor_update() {
  if (pidMode) {
    Motor_pidControlLoop();
  } else {
    Motor_calcPWMFeedForward();
  }

  int go_pin, no_go_pin;

  if (pwm < 0) {
    go_pin = Assignments.in2;
    no_go_pin = Assignments.in1;
  } else {
    go_pin = Assignments.in1;
    no_go_pin = Assignments.in2;
  }

  // Apply rate limits for safety
  unsigned long time = millis();
  float dt = (time - last_updateTime) / 1000;

  int pwmMaxLimit = (dt * 10) + lastPwm;   // Calculate max with rising slew rate
  int pwmMinLimit = (dt * -10) + lastPwm;  // Calculate min with falling slew rate
  pwm = constrain(pwm, pwmMinLimit, pwmMaxLimit);

  analogWrite(no_go_pin, 0);
  analogWrite(go_pin, abs(pwm));
  
  lastPwm = pwm;
}

// Motor_pin_init initalizates pins.
//  honestly, we might want the constuctor to handle this
void MotorControl::Motor_pin_init() {
  pinMode(Assignments.in1, OUTPUT);
  pinMode(Assignments.in2, OUTPUT);
}

void MotorControl::Motor_setPosition(int _position) { position = _position; }

void MotorControl::Motor_pidControlLoop() {

  unsigned long newTime = millis();
  float dt = newTime - last_pidTime;

  // TODO: Might need to factor in wheel radius for rotational speed calculation
  current_velocity = position - lastPositionTicks / dt;
  lastPositionTicks = position;

  float controlSig =
      (int)pidCalculate(pid, goal_velocity, current_velocity, dt);
  controlSig = constrain(controlSig, -255, 255);

  pwm = (int)controlSig;
}

void MotorControl::Motor_calcPWMFeedForward() {
  // TODO: Calculate PWM for goal_velocity
  pwm = round(24.52 * goal_velocity - 0.172);
}

int MotorControl::Motor_getSpeed() { return current_velocity; }

void MotorControl::Motor_enablePIDMode(bool enable) { pidMode = enable; }
