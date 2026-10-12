// Resolved Rates Algorithm (rra) - calculate sequential servo motor values to achieve a specific end effector position and orientation.
// Take in information by passing it by reference, and then change it in the function.
// Returns true if the goal was reached, false if it gave up after params.max_iterations passes.

bool rra(RobotState& state, ConvergenceError& error, BLA::Matrix<3, 1>& Xd, BLA::Matrix<3, 3>& Rd, RRAParams& params) {

  for (int iteration = 0; iteration < params.max_iterations; iteration++) {

    // Forward kinematics: current joint angles (state.q) -> frames 1-6 in state
    kin(state);

    // Jacobian: built from the frames above, so it must run after kin()
    Jacobian(state);

    // Find current position and orientation of end effector
    BLA::Matrix<3, 1> Xc = { state.frames.f6(0, 3), state.frames.f6(1, 3), state.frames.f6(2, 3) };  // current position
    BLA::Matrix<3, 3> Rc = { state.frames.f6(0, 0), state.frames.f6(0, 1), state.frames.f6(0, 2), state.frames.f6(1, 0), state.frames.f6(1, 1), state.frames.f6(1, 2), state.frames.f6(2, 0), state.frames.f6(2, 1), state.frames.f6(2, 2) };  // current orientation

    // Calculate Position error (error.posE)
    BLA::Matrix<1, 1> pos_diff_sq_m = ~(Xd - Xc) * (Xd - Xc);  // (Xd-Xc)^T * (Xd-Xc), as a 1x1 matrix
    float pos_diff_sq = pos_diff_sq_m(0, 0);
    error.posE = sqrt(pos_diff_sq);

    // Calculate orientational error (error.thetaE)
    BLA::Matrix<3, 3> RotE = Rd * ~Rc;                        // rotational error matrix
    float trace_RotE = RotE(0, 0) + RotE(1, 1) + RotE(2, 2);  // trace operation
    error.thetaE = acos((trace_RotE - 1) / 2);                // orientational error

    // *** If position and orientation desired thresholds are hit, we are done
    if (error.posE < params.epsilon_p && error.thetaE < params.epsilon_o) {
      return true;
    }

    // Calculate Nhat (unit vector pointing from current position to desired position)
    float Xd_Xc[3][1];  // find difference between Xd-Xc
    Xd_Xc[0][0] = Xd(0, 0) - Xc(0, 0);
    Xd_Xc[1][0] = Xd(1, 0) - Xc(1, 0);
    Xd_Xc[2][0] = Xd(2, 0) - Xc(2, 0);
    float norm_Xd_Xc = calculate_norm(Xd_Xc);       // norm(Xd-Xc)
    BLA::Matrix<3, 1> nhat = (Xd - Xc) / norm_Xd_Xc;  // Nhat

    // Calculate Vhat (linear speed: vmax when far from the goal, ramping down to vmin as it gets close)
    float vhat;
    float v_ramp = ((params.vmax - params.vmin) * (error.posE - params.epsilon_p)) / (params.epsilon_p * (params.lambda - 1));
    if (error.posE / params.epsilon_p > params.lambda) {
      vhat = params.vmax;
    } else if (error.posE / params.epsilon_p <= params.lambda) {
      vhat = params.vmin + v_ramp;
    }

    // Calculate Pddot (derivative of position desired)
    BLA::Matrix<3, 1> Pddot = nhat * vhat;

    // Calculate W (angular speed: wmax when far from the goal orientation, ramping down to wmin)
    float w;
    float w_ramp = ((params.wmax - params.wmin) * (error.thetaE - params.epsilon_o)) / (params.epsilon_o * (params.lambda - 1));
    if (error.thetaE / params.epsilon_o / params.epsilon_p > params.lambda) {
      w = params.wmax;
    } else if (error.thetaE / params.epsilon_o / params.epsilon_p <= params.lambda) {
      w = params.wmin + w_ramp;
    }

    // Calculate Mhat (unit axis of the rotation needed to fix the orientation error)
    float mhat_scale = 1 / (2 * sin(error.thetaE));
    BLA::Matrix<3, 1> rot_axis_raw = { RotE(2, 1) - RotE(1, 2), RotE(0, 2) - RotE(2, 0), RotE(1, 0) - RotE(0, 1) };
    BLA::Matrix<3, 1> mhat = rot_axis_raw * mhat_scale;

    // Calculate Wd
    BLA::Matrix<3, 1> wd = mhat * w;

    // Calculate Xddot (desired end effector velocity: linear on top, angular below)
    BLA::Matrix<6, 1> Xddot;
    Xddot(0, 0) = Pddot(0, 0);
    Xddot(1, 0) = Pddot(1, 0);
    Xddot(2, 0) = Pddot(2, 0);
    Xddot(3, 0) = wd(0, 0);
    Xddot(4, 0) = wd(1, 0);
    Xddot(5, 0) = wd(2, 0);

    // Jacobian damped pseudo inverse:  Jpinv = J^T * (J*J^T + rho*I)^-1
    float rho = DAMPING_RHO;                      // damping factor
    BLA::Matrix<6, 6> JJt = state.J * ~state.J;   // J * transpose(J)
    for (int i = 0; i < 6; i++) {
      JJt(i, i) = JJt(i, i) + rho;                // add rho * identity
    }
    BLA::Matrix<6, 6> JJt_inv = Inverse(JJt);     // (J*transpose(J) + rho*I)^-1
    BLA::Matrix<6, 6> Jpinv = ~state.J * JJt_inv; // pseudo inverse of jacobian

    // Qddot calculation (joint velocities, rad/s)
    BLA::Matrix<6, 1> qddot = Jpinv * Xddot;

    // Update artificial motor values (state.q)
    float rad_to_deg = 180.0 / pi;
    BLA::Matrix<6, 1> inc = qddot * rad_to_deg;  // increment matrix (deg/s)
    state.q = state.q + inc * params.delta_time;

    // Update real motor values (state.qr)
    for (int i = 0; i < 6; i++) {
      state.qr(i, 0) = state.qr(i, 0) + inc(i, 0) * params.delta_time * US_PER_DEG;  // 1 degree = US_PER_DEG units
    }

    // Command joints (servo motors) to go updated position
    servo1.writeMicroseconds(state.qr(0, 0));
    servo2.writeMicroseconds(state.qr(1, 0));
    servo3.writeMicroseconds(state.qr(2, 0));
    servo4.writeMicroseconds(state.qr(3, 0));
    servo5.writeMicroseconds(state.qr(4, 0));
    servo6.writeMicroseconds(state.qr(5, 0));

  }  // end of iteration loop

  // Ran out of iterations without reaching the goal
  Serial.print("\nrra gave up: goal not reached in ");
  Serial.print(params.max_iterations);
  Serial.print(" iterations. posE = ");
  Serial.print(error.posE);
  Serial.print(", thetaE = ");
  Serial.print(error.thetaE);
  return false;

}  // END OF FUNCTION
