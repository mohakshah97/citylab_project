from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    my_node =  Node(
            package='robot_patrol',
            executable='patrol',
            output='screen',
            emulate_tty=True)

    config_file = LaunchConfiguration('config_file', default='/home/user//.rviz2/cp5.rviz')

    rviz_node =     rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_file],
        output='screen'
    ) 
    config_file = LaunchConfiguration('config_file', default='home/user/cp5.rviz')

    return LaunchDescription([
        my_node,
        rviz_node 
    ])
