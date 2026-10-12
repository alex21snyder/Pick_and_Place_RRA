// robotstate.h - data structures used across the robot program.
// Blueprints only: no memory is used until a variable of one of these types is created.

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <BasicLinearAlgebra.h>

struct Frames {
  BLA::Matrix<4, 4> f1, f2, f3, f4, f5, f6;  // frames 1-6 are always used together
};

struct RobotState {
  BLA::Matrix<6, 1> q;   // artificial (DH-space) joint angles, degrees
  BLA::Matrix<6, 1> qr;  // real (hardware-space) joint values, microseconds
  Frames frames;         // frame 1-6 transformation matrices
  BLA::Matrix<6, 6> J;   // Jacobian
};

struct RRAParams {
  float delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lambda;  // user defined parameters
  int max_iterations;                                                     // give up after this many loop passes
};

struct ConvergenceError {
  float posE, thetaE;
};

#endif
