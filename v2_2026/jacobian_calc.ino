// Jacobian function - build the 6x6 Jacobian (state.J) from the frames calculated by kin().
// Must run after kin(). Layout of the result is pictured at the bottom of this file.

void Jacobian(RobotState& state) {

  // z0-z5 = z-axis of each joint's reference frame (z0 = base frame)
  // p1-p6 = origin (position) of frames 1-6; the base origin is (0,0,0)
  BLA::Matrix<3, 1> z0 = { 0.0, 0.0, 1.0 };
  BLA::Matrix<3, 1> z1 = { state.frames.f1(0, 2), state.frames.f1(1, 2), state.frames.f1(2, 2) };
  BLA::Matrix<3, 1> z2 = { state.frames.f2(0, 2), state.frames.f2(1, 2), state.frames.f2(2, 2) };
  BLA::Matrix<3, 1> z3 = { state.frames.f3(0, 2), state.frames.f3(1, 2), state.frames.f3(2, 2) };
  BLA::Matrix<3, 1> z4 = { state.frames.f4(0, 2), state.frames.f4(1, 2), state.frames.f4(2, 2) };
  BLA::Matrix<3, 1> z5 = { state.frames.f5(0, 2), state.frames.f5(1, 2), state.frames.f5(2, 2) };

  BLA::Matrix<3, 1> p1 = { state.frames.f1(0, 3), state.frames.f1(1, 3), state.frames.f1(2, 3) };
  BLA::Matrix<3, 1> p2 = { state.frames.f2(0, 3), state.frames.f2(1, 3), state.frames.f2(2, 3) };
  BLA::Matrix<3, 1> p3 = { state.frames.f3(0, 3), state.frames.f3(1, 3), state.frames.f3(2, 3) };
  BLA::Matrix<3, 1> p4 = { state.frames.f4(0, 3), state.frames.f4(1, 3), state.frames.f4(2, 3) };
  BLA::Matrix<3, 1> p5 = { state.frames.f5(0, 3), state.frames.f5(1, 3), state.frames.f5(2, 3) };
  BLA::Matrix<3, 1> p6 = { state.frames.f6(0, 3), state.frames.f6(1, 3), state.frames.f6(2, 3) };

  // Rows 1-3 of the Jacobian (linear part), one column per joint: z x (p6 - p_joint)
  BLA::Matrix<3, 1> v0, v1, v2, v3, v4, v5;
  cross(z0, p6, v0);
  cross(z1, p6 - p1, v1);
  cross(z2, p6 - p2, v2);
  cross(z3, p6 - p3, v3);
  cross(z4, p6 - p4, v4);
  cross(z5, p6 - p5, v5);

  // Fill the Jacobian: rows 1-3 = v (linear), rows 4-6 = z (angular)
  for (int i = 0; i < 3; i++) {
    state.J(i, 0) = v0(i);   state.J(i + 3, 0) = z0(i);
    state.J(i, 1) = v1(i);   state.J(i + 3, 1) = z1(i);
    state.J(i, 2) = v2(i);   state.J(i + 3, 2) = z2(i);
    state.J(i, 3) = v3(i);   state.J(i + 3, 3) = z3(i);
    state.J(i, 4) = v4(i);   state.J(i + 3, 4) = z4(i);
    state.J(i, 5) = v5(i);   state.J(i + 3, 5) = z5(i);
  }

}  // end of function

//  Resulting Jacobian (state.J, 6x6):   [ linear velocity  ]  =  J * [ joint rates ]
//                                       [ angular velocity ]
//
//            joint 1  joint 2  joint 3  joint 4  joint 5  joint 6
//             col 0    col 1    col 2    col 3    col 4    col 5
//          |                                                       |
//  row 0   |  v0(0)   v1(0)   v2(0)   v3(0)   v4(0)   v5(0)  |
//  row 1   |  v0(1)   v1(1)   v2(1)   v3(1)   v4(1)   v5(1)  |  <-- linear part:  z x (p6 - p_joint)
//  row 2   |  v0(2)   v1(2)   v2(2)   v3(2)   v4(2)   v5(2)  |
//          |-------------------------------------------------------|
//  row 3   |  z0(0)   z1(0)   z2(0)   z3(0)   z4(0)   z5(0)  |
//  row 4   |  z0(1)   z1(1)   z2(1)   z3(1)   z4(1)   z5(1)  |  <-- angular part: z-axis of the joint
//  row 5   |  z0(2)   z1(2)   z2(2)   z3(2)   z4(2)   z5(2)  |
//
//  Each column belongs to one joint:
//    top 3 rows    = how the end effector's POSITION moves per unit of that joint's motion
//    bottom 3 rows = how the end effector's ORIENTATION rotates per unit of that joint's motion
//  rra() uses this same layout: Xddot rows 0-2 are linear velocity, rows 3-5 are angular velocity.
