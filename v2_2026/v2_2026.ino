/* 6 DOF Robotic Manipulator Control Program

User inputs desired end effector positions and orientations in this main function.
Program uses resolved rates algorithm (rra) to calculate and update new joint positions
every iteration until end effector converges.

by: Alex Snyder
*/


// 1. Library includes first
#include <BasicLinearAlgebra.h>
#include <Servo.h>

// 2. Constants/macros after
#define pi 3.14159265

// 3. Project headers: data structures and named constants
#include "robotstate.h"
#include "config.h"

// 4. Create Servo objects that are global variables
Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7;  

// 5. Define Home Position
const BLA::Matrix<6, 1> Perch = { 1401.0, 995.0, 1511.1, 1500.0, 1500.0, 1500.0 };  // real motor values (microseconds)



void setup() {

  delay(POWER_SETTLE_MS);  // Delay to let power stabilize

  // Define variables here so they are local to this function
  RobotState state;
  RRAParams params;
  ConvergenceError error;
  float pos7; //end effector servo

  // Initialize all parameters before starting robot motion (passed by reference)
  init_robot(state, params, error, pos7);



  //  !!!!!!!!!!!!!!!!!!!!!!    PROGRAM BEGIN     !!!!!!!!!!!!!!!!!!!!!

  // Display Artificial Starting values
  Serial.print("\n\nStarting Q(artificial) values =");
  print_state(state);

  // MOTION 1: command tool center point to go from home to 1st location desired.
  BLA::Matrix<3, 1> Xd = { 188.0, -104.0, 312.0 };              // position desired
  BLA::Matrix<3, 3> Rd = { 0, 0, 1.0, 0, 1.0, 0, -1.0, 0, 0 };  // orientation desired
  if (!rra(state, error, Xd, Rd, params)) {
    return;  // motion failed, stop here
  }
  close_gripper(pos7);  // actuate tool
  
  // Display Artificial Q values after Motion 1
  Serial.print("\n\nAfter Motion 1 Q(artificial) values =");
  print_state(state);


  // MOTION 2: command end effector to go from 1st location to 2nd location.
  Xd = { 104, 0, 394.0 };                                     // position desired
  Rd = { 0.7071, 0, 0.7071, 0, 1.0, 0, -0.7071, 0, 0.7071 };  // orientation desired
  if (!rra(state, error, Xd, Rd, params)) {
    return;  // motion failed, stop here
  }
  // Display Artificial Q values after Motion 2
  Serial.print("\n\nAfter Motion 2 Q(artificial) values =");
  print_state(state);

  // MOTION 3: command end effector to go from 2nd location to 3rd location.
  Xd = { 190.0, 104.0, 305.0 };               // position desired
  Rd = { 0, 0, 1.0, 0, 1.0, 0, -1.0, 0, 0 };  // orientation desired
  params.wmin = .75;
  params.wmax = 1.375;  // wmin, wmax adjusted for optimal speed
  if (!rra(state, error, Xd, Rd, params)) {
    return;  // motion failed, stop here
  }
  open_gripper(pos7);  // actuate tool
  // Display Artificial Q values after Motion 3
  Serial.print("\n\nAfter Motion 3 Q(artificial) values =");
  print_state(state);

  // MOTION 4: go to home
  BLA::Matrix<6, 1> qdiff = Perch - state.qr;
  float amount = HOME_STEPS;  // number of steps to go from position 3 to home
  qdiff = qdiff / amount;     // 1 step closer to home
  for (int j = 0; j < amount; j++) {
    state.qr = state.qr + qdiff;
    servo1.writeMicroseconds(state.qr(0, 0));
    servo2.writeMicroseconds(state.qr(1, 0));
    servo3.writeMicroseconds(state.qr(2, 0));
    servo4.writeMicroseconds(state.qr(3, 0));
    servo5.writeMicroseconds(state.qr(4, 0));
    servo6.writeMicroseconds(state.qr(5, 0));
    delay(HOME_STEP_DELAY_MS);
  }

  // Confirm motors are exactly at home.
  servo1.write(Perch(0, 0));
  servo2.write(Perch(1, 0));
  servo3.write(Perch(2, 0));
  servo4.write(Perch(3, 0));
  servo5.write(Perch(4, 0));
  servo6.write(Perch(5, 0));
  servo7.write(pos7);

  // Display Artificial Q values after Motion 4
  Serial.print("\n\nAfter Motion 4 Q(artificial) values =");
  print_state(state);
  Serial.print("\n");
  Serial.print("done :)");
}  //  END OF PROGRAM

void loop() {

  while (1 == 1) {  // do nothing loop
  }
}



// -ALEX 26
