/**
 * @file MotorControl.h
 * @author Domenic Marcelli R&D Team
 * @brief This the header file for the motor controls.
 * @date 2023-10-27
 *
 * @copyright Copyright (c) 2024
 *
 */

 #ifndef MOTORCONTROL_H_
 #define MOTORCONTROL_H_
 
 // #include "arduino_freertos.h"
 #include "Arduino.h"
 #include "ros.h"
 #include <avr/io.h>
 #include <avr/pgmspace.h>
 #include <util/PID.hpp>
 
 #define tskPID_PRIORITY 7 // can be changed. Up for discussion
 
 #define PID_LOOP_PERIOD 50
 
 /**
  * @brief Class representing a Motor
  */
 class MotorControl {
 
 public:
   /**
    * @brief Construct a new Motor Control object
    *
    * @param pin1 pin 1 of motor controller
    * @param pin2 pin 2 of motor controller
    */
   MotorControl(int pin1, int pin2);
 
   /**
    * @brief Set up motor PID controller
    *
    * @param P Proportional gain
    * @param I Integral gain
    * @param D Derivative gain
    */
   void Motor_setPIDParams(float P, float I, float D);
 
   /**
    * @brief Enables motors to begin running at a speed setpoint
    *
    * @param newSpeed Desired motor speed
    */
   void Motor_setGoalSpeed(float newSpeed);
 
   void Motor_update();
 
   /**
    * @brief Initialize motor pins
    *
    */
   void Motor_pin_init();
 
   /**
    * @brief Motor PID loop function
    */
   void Motor_pidControlLoop();
 
   void Motor_calcPWMFeedForward();
 
   void Motor_setPosition(int ticks);
 
   /**
    * @brief Get the motor speed
    *
    * @return motor speed as an int
    */
   int Motor_getSpeed();
 
   void Motor_enablePIDMode(bool enable);
 
 private:
   /**
    * @brief Boolean indicating if Motor is running PID control
    */
   bool pidMode;
 
   /**
    * @brief Motor pwm
    */
   float pwm;
 
   /**
    * @brief Motor instantaneous angular velocity
    *
    */
   float current_velocity;
 
   /**
    * @brief Target angular velocity
    */
   float goal_velocity;
 
   /**
    * @brief Struct for holding motor pins
    */
   struct _PinAssign {
     int in1;
     int in2;
   };
   typedef _PinAssign PinAssign;
 
   /**
    * @brief Pins
    */
   PinAssign Assignments;
 
   /**
    * @brief Motor PID
    *
    */
   PID pid;
 
   unsigned long last_pidTime;
   unsigned long last_updateTime;
 
   int lastPositionTicks;
   int position;
   float lastPwm;
 };
 // All the variables needed to control the motors
 
 #endif
 