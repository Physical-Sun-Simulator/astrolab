import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'dt.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('astrolab'),
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='astrolab',
            executable='visualization',
            name='visualization',
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', 'install/astrolab/share/astrolab/dt.rviz'],
            name='rviz'),
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
