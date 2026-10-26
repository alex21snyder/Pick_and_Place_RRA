"""Show the arm in RViz with a slider window to move each joint.

Run:  ros2 launch /full/path/to/v3_2026_ros2/launch/display.launch.py

Joint slider values are in radians (ROS convention). The Arduino code's q is in degrees,
so q = 30 degrees  <->  slider 0.5236.

If rviz/arm.rviz exists (File -> Save Config As in RViz), RViz opens with it already set up.
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(here, "..", "urdf", "arm.urdf")
    rviz_config = os.path.normpath(os.path.join(here, "..", "rviz", "arm.rviz"))
    with open(urdf_path) as f:
        robot_description = f.read()
    # The URDF refers to CAD meshes as file://MESH_DIR/...; point that at this folder's meshes/.
    mesh_dir = os.path.normpath(os.path.join(here, "..", "meshes"))
    robot_description = robot_description.replace("MESH_DIR", mesh_dir)

    rra_node = os.path.normpath(os.path.join(here, "..", "rra_py", "rra_node.py"))
    slider = LaunchConfiguration("slider")
    demo = LaunchConfiguration("demo")

    # Use the saved RViz setup if it exists; otherwise RViz opens with its defaults.
    rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        DeclareLaunchArgument("slider", default_value="false"),
        DeclareLaunchArgument("demo", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # Sliders (Stage 1): launch with  slider:=true  to use them instead of the algorithm.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(slider),
        ),
        # Stage 2: resolved rates algorithm drives /joint_states. Plays Motions 1-3 unless demo:=false.
        Node(
            executable=sys.executable,
            arguments=[rra_node, "--ros-args", "-p", ["demo:=", demo]],
            condition=UnlessCondition(slider),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=rviz_args,
        ),
    ])
