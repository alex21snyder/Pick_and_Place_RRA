// Kinematics function - calculate frames 1-6 based on the current artificial servo motor values (state.q).
// Take in information by passing it by reference, and then change it in the function.
// The Jacobian is built separately by Jacobian(), which must run after this.

void kin(RobotState& state) {

  // DH table (angles in degrees, lengths in mm)
  //                        theta                 d     r      alpha
  BLA::Matrix<6, 4> DH = { state.q(0, 0),         35.5,  -4,    90,
                           state.q(1, 0) + 270,   -8.7,  -269,  180,
                           state.q(2, 0) + 270,   -8.7,  0,     90,
                           state.q(3, 0),         110.2, -5.5,  90,
                           state.q(4, 0),         25.5,  0,     270,
                           state.q(5, 0),         55,    11.5,  0 };

  // Homogeneous transformations
  BLA::Matrix<4, 4> a1 = dh_transform(DH(0, 0), DH(0, 1), DH(0, 2), DH(0, 3));
  BLA::Matrix<4, 4> a2 = dh_transform(DH(1, 0), DH(1, 1), DH(1, 2), DH(1, 3));
  BLA::Matrix<4, 4> a3 = dh_transform(DH(2, 0), DH(2, 1), DH(2, 2), DH(2, 3));
  BLA::Matrix<4, 4> a4 = dh_transform(DH(3, 0), DH(3, 1), DH(3, 2), DH(3, 3));
  BLA::Matrix<4, 4> a5 = dh_transform(DH(4, 0), DH(4, 1), DH(4, 2), DH(4, 3));
  BLA::Matrix<4, 4> a6 = dh_transform(DH(5, 0), DH(5, 1), DH(5, 2), DH(5, 3));

  // Frames calculations
  state.frames.f1 = a1;
  state.frames.f2 = state.frames.f1 * a2;
  state.frames.f3 = state.frames.f2 * a3;
  state.frames.f4 = state.frames.f3 * a4;
  state.frames.f5 = state.frames.f4 * a5;
  state.frames.f6 = state.frames.f5 * a6;

}  // END OF FUNCTION
