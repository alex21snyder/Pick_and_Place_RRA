# Pick_and_Place_RRA

A resolved-rates motion controller for a 6-DOF 3D-printed robotic arm run by an Arduino Mega, plus a ROS 2 simulation of the same arm.

**Video (real arm):** [Arm picking and placing an object](https://www.youtube.com/shorts/7NZ8k-jHHZM)
**Video (simulation):** [RViz model of the arm](https://youtu.be/b6Oi5uShVIs)

## How it works

The algorithm is resolved-rates motion control, taken from course material at Stevens Institute of Technology (Resolved Rates Algorithm for Serial Robots). My implementation of it, the DH model of this arm, the tuning, and the hardware are my own.

Each motion is defined by a target end-effector position and orientation. The algorithm repeats these steps until the end effector is within tolerance:

1. Compute the current end-effector pose with Denavit-Hartenberg forward kinematics.
2. Build the Jacobian.
3. Compute the desired end-effector velocity.
4. Solve for joint velocities using a damped pseudo-inverse of the Jacobian.
5. Update the joint angles and command the servos.

## Repository layout

- [`v1_2023/`](v1_2023/): the original version, written from scratch in 2023.
- [`v2_2026/`](v2_2026/): a 2026 restructure of the Arduino C++ (structs, separate kinematics and Jacobian functions, named constants, an iteration limit).
- [`v3_2026_ros2/`](v3_2026_ros2/): a ROS 2 / RViz simulation of the arm, built from my SolidWorks CAD, with a Python port of the algorithm.

Each version folder has its own README.

## Note on AI assistance

The implementation, kinematics model of the arm, tuning, hardware and CAD are my work. The algorithm itself is from the Stevens course material above. The v2 restructure was done in 2026 with AI assistance (Claude) for code structure and readability, and tested on the arm. The v3 ROS 2 port and tooling were written with Claude Code and checked against the arm's recorded output.
