from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[{
                "robot_description": "arm06.urdf",
                "robot_description_semantic": "arm06.srdf",
                # Optional configurations
                "kinematics.yaml": "kinematics.yaml",
                # Other configs as needed
            }]
        ),
    ])




# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("arm06", package_name="arm06_moveit").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)
