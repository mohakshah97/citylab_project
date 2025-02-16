from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
        service_server_node =    Node(
            package="robot_patrol",
            executable ="direction_service",
            output = "screen"
        )

        patrol_node = Node(
        package='robot_patrol',
        executable = "patrol_with_service",
        output = "screen"
        )

        config_file = LaunchConfiguration('config_file', default='/home/user/ros2_ws/src/citylab_project/robot_patrol/rviz/cp5.rviz')

        rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_file],
        output='screen'
    ) 

        return LaunchDescription([
        service_server_node,
        patrol_node, 
        rviz_node
        ])