from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='astrolab',
            executable='arm',
            name='arm',
            output='screen'),
        Node(
            package='astrolab',
            executable='table',
            name='table',
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'src/astrolab/webserver/webserver.py'],
            name='webserver',
            output='screen'),
    ])
