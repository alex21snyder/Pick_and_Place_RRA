# Stage 2: resolved rates algorithm in ROS 2

- `rra.py`      numpy port of the v2 C++ (rra, Jacobian, kin). Run `python3 rra.py` -> must print MATCHES ARDUINO.
- `rra_node.py` ROS 2 node. /goal_pose (PoseStamped, meters, base_link) in -> /joint_states out.

Run (arm plays Motions 1-3):  ros2 launch ~/v3_2026_ros2/launch/display.launch.py
Wait for a goal instead:      ... display.launch.py demo:=false
Old sliders:                  ... display.launch.py slider:=true
