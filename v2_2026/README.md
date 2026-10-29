# v2_2026: Arduino C++ controller

Built on my original 2023 code (`v1_2023/`). The algorithm, kinematics model and hardware are unchanged. This version restructures the code and was refined with AI assistance (Claude) for structure and readability, then tested on the arm.

## What it does

On startup it runs one pick-and-place routine, printing the joint angles to the Serial Monitor after each motion:

1. **Motion 1:** from the home position to the pick location (188, -104, 312 mm) with the gripper level, then closes the gripper.
2. **Motion 2:** lifts to (104, 0, 394 mm), tilting the tool 45 degrees.
3. **Motion 3:** moves to the place location (190, 104, 305 mm) with the gripper level, then opens the gripper.
4. **Motion 4:** returns to home in a straight line in joint space.

## Files

- `v2_2026.ino`: main program. Defines the home position (`Perch`, in servo microseconds) and runs the four motions.
- `rra.ino`: the resolved-rates loop. Each pass computes the pose error, the desired end-effector velocity, the damped pseudo-inverse of the Jacobian, and the new joint angles, then commands the servos. Gives up after `max_iterations` passes.
- `kin_function.ino`: forward kinematics. Holds the DH table and builds frames 1-6.
- `homogeneous_transformation.ino`: builds one 4x4 DH transform.
- `jacobian_calc.ino`: builds the 6x6 Jacobian from the frames.
- `cross_prod.ino`, `norm_calc.ino`: small vector helpers.
- `init.ino`: starting values, servo pins and the RRA tuning parameters.
- `print_state.ino`: prints the joint angles.
- `robotstate.h`: the structs (`RobotState`, `Frames`, `RRAParams`, `ConvergenceError`).
- `config.h`: named constants (pins, timing, gripper positions, microseconds per degree, damping factor).
- `close_gripper.ino`: closes the gripper to pick up the object, with a settling delay before and a pick delay after.
- `open_gripper.ino`: opens the gripper to release the object, with a settling delay before and a place delay after.

## Hardware and libraries

- Arduino Mega, six joint servos, and a gripper servo
- [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra)
- Servo library (included with the Arduino IDE)
