from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    action_node =  Node(
            package='robot_patrol',
            executable='go_to_pose_action',
            output='screen',
            emulate_tty=True)

    config_file = LaunchConfiguration('config_file', default='/home/user/ros2_ws/src/citylab_project/robot_patrol/rviz/cp5.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_file],
        output='screen'
    ) 
    # config_file = LaunchConfiguration('config_file', default='home/user/cp5.rviz')

    return LaunchDescription([
        action_node,
        rviz_node 
    ])