# Pick_and_Place_RRA

Resolved Rates Motion Control algorithm running on a 6 degree-of-freedom robotic arm. The arm is 3D printed and controlled by an Arduino Mega.

**Video:** [Arm picking and placing an object](https://www.youtube.com/shorts/7NZ8k-jHHZM)

## How it works

Each motion is given as a target end effector position and orientation. The resolved rates algorithm (RRA) repeats the following until the end effector is within tolerance of the target:

1. Forward kinematics (Denavit-Hartenberg) to find the current end effector pose.
2. Build the Jacobian.
3. Compute the desired end effector velocity.
4. Solve for joint velocities with a damped pseudo-inverse of the Jacobian.
5. Update the joint angles and command the servos.

## Repository layout

- `v1_2023/` - original version, written from scratch in 2023.
- `v2_2026/` - 2026 refactor: data kept in structs, separate functions for kinematics and the Jacobian, named constants, and an iteration limit so the algorithm cannot run forever.

To run v2, open `v2_2026/v2_2026.ino` in the Arduino IDE. The other files open automatically as tabs.

## Hardware and libraries

- Arduino Mega, six joint servos plus a gripper servo
- [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra)
- Servo (included with the Arduino IDE)

## Note on AI assistance

The algorithm, kinematics model and hardware are my original 2023 work. The v2 refactor was done in 2026 with AI assistance (Claude) for code structure and readability, and tested on the arm.
