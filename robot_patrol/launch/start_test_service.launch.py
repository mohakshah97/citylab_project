from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

        test_service_node = Node(
        package='robot_patrol',
        executable = "test_service",
        output = "screen"
        
        )

        return LaunchDescription([
        test_service_node 
    ])