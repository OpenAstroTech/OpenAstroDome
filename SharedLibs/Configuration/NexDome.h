#ifndef NEXDOME_H
#define NEXDOME_H

#include <Arduino.h>
#include "Constants.h"

/*
 * limits.h appears to have values that are not consistent
 * with reality.
 */

constexpr int32_t MaxStepPosition = 2000000000L;
constexpr int32_t MinStepPosition = -2000000000L;

#define MOTOR_TYPE  (DC_MOTOR) // See Constants.h for options
// Motor Parameters
#define MICROSTEPS_PER_STEP (8) // Should match DIP switches on stepper driver
#define MIN_SPEED (250)         // Minimum speed that can be timed by the hardware timer
#define MIN_RAMP_TIME (100)     // Minimum ramp up/down time in milliseconds

// DC Motor defaults
#define MOTOR_BOARD     (MOTOR_CONTROLLER_SHIELDMD10)
#define ENCODER_PIN_A    (2)   // Encoder
#define ENCODER_PIN_B    (3)   // Encoder
#define MOTOR_MIN_PWM   (60)    // Minimum PWM setting needed to move the motor
// Acceleration PID constants
const float DCMOTOR_kp_A = 0.00;
const float DCMOTOR_ki_A = 0.2;
const float DCMOTOR_kd_A = 0.00;
// Running PID constants
const float DCMOTOR_kp_R = 0.5;
const float DCMOTOR_ki_R = 0.0;
const float DCMOTOR_kd_R = 0.02;
// BTS7960
#define MOTOR_ENABLE_PIN_L  (7)
#define MOTOR_ENABLE_PIN_R  (8)
#define MOTOR_PWM_PIN_L (9)
#define MOTOR_PWM_PIN_R (10)
// SHEILDMD10
#define MOTOR_DIRECTION_PIN (8)
#define MOTOR_PWM_PIN (9)

// Stepper Motor defaults
#define MOTOR_STEP_PIN (7) //12
//#define MOTOR_DIRECTION_PIN (8) //11
#define MOTOR_ENABLE_PIN (9) //10
#define MOTOR_RAMP_TIME (1500) // milliseconds to accelerate to full speed
#define MOTOR_MAX_SPEED (25000 * MICROSTEPS_PER_STEP)
#define ROTATOR_DEFAULT_SPEED (600 * MICROSTEPS_PER_STEP)
#define SHUTTER_DEFAULT_SPEED (800 * MICROSTEPS_PER_STEP)
#define SHUTTER_FULL_OPEN_DEFAULT (46000UL * MICROSTEPS_PER_STEP)
#define SHUTTER_LIMIT_STOPPING_DISTANCE (100 * MICROSTEPS_PER_STEP)
//#define ROTATOR_FULL_REVOLUTION_MICROSTEPS (440640)
#define ROTATOR_FULL_REVOLUTION_MICROSTEPS (10000)
#define ROTATOR_MAX_POSITION (MaxStepPosition)
#define ROTATOR_HOME_POSITION (0)
//#define ROTATOR_DEFAULT_DEADZONE (75 * MICROSTEPS_PER_STEP)	// default dead-zone in microsteps (~0.5°)
#define ROTATOR_DEFAULT_DEADZONE (50)	// default dead-zone in microsteps (~0.5°)

#define HOST_SERIAL_RX_BUFFER_SIZE (16) // Receive buffer for PC/USB communications

// Other hardware assignments
#define HOME_INDEX_PIN (13) //PIN13
#define OPEN_LIMIT_SWITCH_PIN (12) //PIN3
#define CLOSED_LIMIT_SWITCH_PIN (11) //PIN2
#define OPEN_BUTTON_PIN (A1) //PIN5
#define CLOSE_BUTTON_PIN (A2) //PIN6
#define CLOCKWISE_BUTTON_PIN (OPEN_BUTTON_PIN)
#define COUNTERCLOCKWISE_BUTTON_PIN (CLOSE_BUTTON_PIN)
#define RAIN_SENSOR_PIN (A3) //PIN7

#endif // NEXDOME_H
