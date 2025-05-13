from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_perception',
            namespace='perception',
            executable='follow2',
            name='turtlebot',
            emulate_tty=True,
        ),
        Node(
            package='robot_perception',
            namespace='perception',
            executable='avoid2',
            name='turtlebot',
            emulate_tty=True, # pour pas bloquer les fenetres
        ),
        Node(
            package='robot_navigation',
            namespace="navigation",
            executable='navigation',
            name='turtlebot',
            
        )
    ])