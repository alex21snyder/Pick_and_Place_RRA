// config.h - named constants for the robot program (hardware, timing and unit values).
// Tuning values for the resolved rates algorithm live in RRAParams (see init.ino), not here.

#ifndef CONFIG_H
#define CONFIG_H

// Serial
constexpr long SERIAL_BAUD = 9600;

// Servo output pins
constexpr int SERVO1_PIN = 11;
constexpr int SERVO2_PIN = 2;
constexpr int SERVO3_PIN = 3;
constexpr int SERVO4_PIN = 4;
constexpr int SERVO5_PIN = 5;
constexpr int SERVO6_PIN = 6;
constexpr int SERVO7_PIN = 7;  // end effector (gripper)

// Unit conversion: 1 degree of joint motion = 9 microseconds of servo command
constexpr float US_PER_DEG = 9.0;

// Gripper (servo 7) positions, passed to servo7.write()
constexpr float GRIPPER_OPEN = 30.0;
constexpr float GRIPPER_CLOSED = 63.0;

// Delays (milliseconds)
constexpr int POWER_SETTLE_MS = 3000;   // let power stabilize at startup
constexpr int GRIPPER_PAUSE_MS = 500;   // pause around opening / closing the gripper
constexpr int HOME_STEP_DELAY_MS = 5;   // pause between steps while returning home

// Motion 4: number of small steps used to go from position 3 back to home
constexpr int HOME_STEPS = 2000;

// Resolved rates algorithm: damping factor (rho) for the damped pseudo inverse
constexpr float DAMPING_RHO = 1.0;

#endif
