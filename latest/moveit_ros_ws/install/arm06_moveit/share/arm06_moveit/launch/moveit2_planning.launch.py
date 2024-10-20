from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm06", package_name="arm06_moveit").to_moveit_configs()

    # Disable the occupancy map monitor explicitly
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"monitoring_plugin": ""},  # Disabling sensor monitoring plugin
            {"octomap_monitor/point_cloud_sensor": "false"},  # Explicitly disable octomap sensor
            {"octomap_monitor/occupancy_map_monitor": "false"},  # Disable occupancy map monitor
        ],
    )

    return LaunchDescription([move_group_node])
