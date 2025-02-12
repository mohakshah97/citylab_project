from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

        service_server_node =    Node(
            package="robot_patrol",
            executable ="direction_service",
            output = "screen"
        )

        return LaunchDescription([
        service_server_node
        ])