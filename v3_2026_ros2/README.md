# v3: ROS 2 / RViz model of the 6-DOF arm

A simulation of the 6-DOF robotic manipulator from `v1_2023/` and `v2_2026/`, built in ROS 2. The arm is modeled from my own SolidWorks CAD, and the same resolved-rates algorithm that runs on the Arduino drives it in RViz.

**Video:** [RViz simulation](https://youtu.be/b6Oi5uShVIs)

## What it does

- Loads the arm in RViz from a URDF generated from the DH table. Each link uses an STL exported from my CAD assembly.
- `rra_py/rra.py` is a numpy port of the v2 C++ (kinematics, Jacobian, damped pseudo-inverse). Running it prints the joint angles for Motions 1-3 and checks them against the angles the real arm printed. They match.
- `rra_py/rra_node.py` is a ROS 2 node that plays Motions 1-3 and the return home, publishing `/joint_states` every 28 ms. It can also take a goal pose on `/goal_pose`.

## Run it

Requires ROS 2 (developed on Lyrical Luth, Ubuntu 26.04) and numpy.

```
source /opt/ros/lyrical/setup.bash
ros2 launch ~/v3_2026_ros2/launch/display.launch.py                  # demo: Motions 1-3, then home
ros2 launch ~/v3_2026_ros2/launch/display.launch.py slider:=true     # move each joint by hand (radians)
ros2 launch ~/v3_2026_ros2/launch/display.launch.py demo:=false      # wait for a PoseStamped on /goal_pose
```

Check the algorithm port against the Arduino results:

```
cd rra_py && python3 rra.py        # prints MATCHES ARDUINO
cd tools && python3 check_fk.py    # URDF vs DH forward kinematics
```

Regenerate the URDF after changing the DH table or meshes:

```
cd tools && python3 make_urdf.py --no-sticks
```

## Layout

- `meshes/`: one STL per link, exported from the CAD assembly at the straight-up home pose
- `urdf/`: `arm.urdf` (generated, do not edit by hand)
- `launch/`: `display.launch.py`
- `rra_py/`: the algorithm port and the ROS 2 node
- `tools/`: `dh.py` (DH tables), `make_urdf.py`, `check_fk.py`

## Notes

- The 3D model is reverse-engineered from the arm I already built, with CAD measurements good to about 5 mm. A few CAD offsets (for example, servo 2 and 3 at x = +4 mm) differ from the DH table, so the model uses its own copy of the table (`DH_CAD`). The algorithm uses the original table, so the motions match the Arduino.
- This is a simulation. It does not send commands to the real arm.

## AI assistance

The algorithm is from the Stevens Institute of Technology course material credited in the main README. The DH model, hardware and CAD are my own work. The ROS 2 port, URDF generator and checking scripts were written with AI assistance (Claude Code) and checked against the arm's recorded output.
