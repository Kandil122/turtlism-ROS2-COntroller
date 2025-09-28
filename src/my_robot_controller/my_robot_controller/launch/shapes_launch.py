from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim"
        ),
        Node(
            package="my_robot_controller",
            executable="shape_node",
            name="shape_node"
        ),
        Node(
            package="my_robot_controller",
            executable="turtle_commander",
            name="turtle_commander"
        ),
    ])
