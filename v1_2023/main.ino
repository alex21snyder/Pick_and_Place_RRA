/* 6 DOF Robotic Manipulator Control Program

User inputs desired end effector positions and orientations in this main function.
Program uses resolved rates algorithm (rra) to calculate and update new joint positions
every iteration until end effector converges.

by: Alex Snyder
11/17/23
*/
#include <Servo.h>
#define pi 3.14159265
#include <BasicLinearAlgebra.h>

using namespace BLA;
Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7;  //create servo object to control a servo


void setup() {
  delay(3000);
  Serial.begin(9600);

  // ACTUAL STARTING POSITION of each motor. These values have no units.
  float pos1 = 1401.0, pos2 = 995.0, pos3 = 1511.1, pos4 = 1500.0, pos5 = 1500.0, pos6 = 1500.0, pos7 = 30.0;  //pos 7 is end effector

  // ARTIFICIAL POSITION of each motor. These are used for calculations and are in degrees.
  float apos1 = 0.0, apos2 = 0.0, apos3 = 0.0, apos4 = 0.0, apos5 = 0.0, apos6 = 0.0;

  // Command each motor to go to starting position before .attach to prevent random servo jerking when code is initially uploaded.
  servo1.writeMicroseconds(pos1);
  servo2.writeMicroseconds(pos2);
  servo3.writeMicroseconds(pos3);
  servo4.writeMicroseconds(pos4);
  servo5.writeMicroseconds(pos5);
  servo6.writeMicroseconds(pos6);
  servo7.write(pos7);

  // Designate each motor to an output pin.
  servo1.attach(11);
  servo2.attach(2);
  servo3.attach(3);
  servo4.attach(4);
  servo5.attach(5);
  servo6.attach(6);
  servo7.attach(7);

  // Initialize all matricies using the "<BasicLinearAlgebra.h>" notation.
  BLA::Matrix<6, 1> qr = { pos1, pos2, pos3, pos4, pos5, pos6 };       //real motor values
  BLA::Matrix<6, 1> q = { apos1, apos2, apos3, apos4, apos5, apos6 };  //artificial motor values for calculations
  BLA::Matrix<4, 4> frames_1;                                          // frame 1
  BLA::Matrix<4, 4> frames_2;                                          // frame 2
  BLA::Matrix<4, 4> frames_3;                                          // frame 3
  BLA::Matrix<4, 4> frames_4;                                          // frame 4
  BLA::Matrix<4, 4> frames_5;                                          // frame 5
  BLA::Matrix<4, 4> frames_6;                                          // frame 6
  BLA::Matrix<6, 6> J;                                                 //jacobian

  // Fill all matricies with 0's to start.
  frames_1.Fill(0);
  frames_2.Fill(0);
  frames_3.Fill(0);
  frames_4.Fill(0);
  frames_5.Fill(0);
  frames_6.Fill(0);
  J.Fill(0);

  // Initialize RRA parameters
  float PosE = 0.0;    //Position error of end effector
  float ThetaE = 0.0;  //Orientation error of end effector



  //  !!!!!!!!!!!!!!!!!!!!!!    PROGRAM BEGIN     !!!!!!!!!!!!!!!!!!!!!

  // Starting Parameters (user defined)
  float delta_time = .028;  //s
  float vmin = 7;           //m/s
  float vmax = 24;          //m/s
  float wmin = .30;         //rad/s
  float wmax = .55;         //rad/s
  float epsilon_p = 1;      //m
  float epsilon_o = .1;     //rad
  float lamda = 5;          //unitless

  //Display Artifitial Starting values
  Serial.print("\n\nStarting Q(artificial) values =\n");
  Serial << q;  // display q matrix
  Serial.print("\n");


  // MOTION 1: command end effector to go from home to 1st location desired.
  BLA::Matrix<3, 1> Xd = { 188.0, -104.0, 312.0 };              //position desired
  BLA::Matrix<3, 3> Rd = { 0, 0, 1.0, 0, 1.0, 0, -1.0, 0, 0 };  //orientation desired
  rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);
  delay(500);
  pos7 = 63;
  servo7.write(pos7);  // close the gripper

  // MOTION 2: command end effector to go from 1st location to 2nd location.
  Xd = { 104, 0, 394.0 };                                     //position desired
  Rd = { 0.7071, 0, 0.7071, 0, 1.0, 0, -0.7071, 0, 0.7071 };  //orientation desired
  rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);

  // MOTION 3: command end effector to go from 2nd location to 3rd location.
  Xd = { 190.0, 104.0, 305.0 };               //position desired
  Rd = { 0, 0, 1.0, 0, 1.0, 0, -1.0, 0, 0 };  //orientation desired
  wmin = .75;
  wmax = 1.375;  //wmin, wmax adjusted for optimal speed
  rra(q, qr, frames_1, frames_2, frames_3, frames_4, frames_5, frames_6, J, PosE, ThetaE, Xd, Rd, delta_time, vmin, vmax, wmin, wmax, epsilon_p, epsilon_o, lamda);
  delay(500);
  pos7 = 30;
  servo7.write(pos7);  //open end effector
  delay(500);

  // MOTION 4: command end effector to go from 3 position to home.
  BLA::Matrix<6, 1> qh = { pos1, pos2, pos3, pos4, pos5, pos6 };  //origonal motor values
  BLA::Matrix<6, 1> qdiff = qh - qr;
  Serial.print("\n");
  Serial << qdiff;
  float amount = 2000.0;   // amount of steps to go from position 3 to home
  qdiff = qdiff / amount;  // 1 step closer to home
  for (int j = 0; j < amount; j++) {
    qr = qr + qdiff;
    servo1.writeMicroseconds(qr(0, 0));  // starting position when power is turned on
    servo2.writeMicroseconds(qr(1, 0));
    servo3.writeMicroseconds(qr(2, 0));
    servo4.writeMicroseconds(qr(3, 0));
    servo5.writeMicroseconds(qr(4, 0));
    servo6.writeMicroseconds(qr(5, 0));
    delay(5);
  }
  Serial.print("\n");
  Serial.print("done");

  // Make sure motors are exactly at home.
  pos1 = 79.0;  //degrees
  pos2 = 44.0;  //degrees
  pos3 = 91.0;  //degrees
  pos4 = 90.0;  //degrees
  pos5 = 90.0;  //degrees
  pos6 = 90.0;  //degrees
  pos7 = 30.0;  //degrees
  servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);
  servo5.write(pos5);
  servo6.write(pos6);
  servo7.write(pos7);
}  //  END OF PROGRAM

void loop() {

  while (1 == 1) {  // do nothing loop
  }
}



// -ALEX 23
