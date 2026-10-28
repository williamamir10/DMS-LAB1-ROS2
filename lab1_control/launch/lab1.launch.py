from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='lab1_control',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
        Node(
            package='lab1_control',
            executable='square_server',
            name='square_server',
            output='screen'
        ),
    ])
