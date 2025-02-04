import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    enable_controller = LaunchConfiguration('enable_controller', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')    


    package_dir = get_package_share_directory('ik_pkg')

    urdf_file_name = 'arm11.urdf'
    urdf = os.path.join(package_dir, urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_path = os.path.join(package_dir, 'viz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'enable_controller',
            default_value='false',
            description='Enable controller inputs'),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set to true to launch RViz'),


        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='ik_pkg',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[
                {'enable_controller': enable_controller}
            ]),
                # Conditionally launch RViz based on the launch_rviz flag
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(launch_rviz),  # Only launch if 'launch_rviz' is set to 'true'
        ),
    ])
