/*
Reset parameters and assign starting values to variables.
*/



void init_robot(RobotState& state, RRAParams& params, ConvergenceError& error, float& pos7) {
  Serial.begin(SERIAL_BAUD);  // initialize serial communication, the channel arduino uses to send text back to computer

  // build the robot initial state using the RobotState struct
  state.qr = Perch;                            // real motor values (microseconds)
  state.q = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // artificial motor values (degrees)

  // pos7 stays separate - it's the end effector, not one of the 6 arm joints
  pos7 = GRIPPER_OPEN;

  // frames and Jacobian start filled with 0
  state.frames.f1.Fill(0);
  state.frames.f2.Fill(0);
  state.frames.f3.Fill(0);
  state.frames.f4.Fill(0);
  state.frames.f5.Fill(0);
  state.frames.f6.Fill(0);
  state.J.Fill(0);

  // Command each motor to go to starting position before .attach to prevent random servo jerking when code is initially uploaded.
  servo1.writeMicroseconds(state.qr(0, 0));
  servo2.writeMicroseconds(state.qr(1, 0));
  servo3.writeMicroseconds(state.qr(2, 0));
  servo4.writeMicroseconds(state.qr(3, 0));
  servo5.writeMicroseconds(state.qr(4, 0));
  servo6.writeMicroseconds(state.qr(5, 0));
  servo7.write(pos7);

  // Designate each motor to an output pin.
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);
  servo6.attach(SERVO6_PIN);
  servo7.attach(SERVO7_PIN);

  // Robot Error - live convergence tracking, updated by rra() every iteration
  error.posE = 0.0;
  error.thetaE = 0.0;

  // RRA Parameters - fixed tuning inputs (user defined)
  params.delta_time = .028;      // s
  params.vmin = 7;               // m/s
  params.vmax = 24;              // m/s
  params.wmin = .30;             // rad/s
  params.wmax = .55;             // rad/s
  params.epsilon_p = 1;          // m
  params.epsilon_o = .1;         // rad
  params.lambda = 5;             // unitless
  params.max_iterations = 1500;  // slowest motion converges in ~455 passes; cap is ~3x that
}
