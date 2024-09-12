import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(name="host", default_value="localhost"),
            DeclareLaunchArgument(name="port", default_value="2000"),
            DeclareLaunchArgument(name="timeout", default_value="10"),
            DeclareLaunchArgument(name="role_name", default_value="ego_vehicle"),
            DeclareLaunchArgument(name="vehicle_filter", default_value="vehicle.*"),
            DeclareLaunchArgument(name="spawn_point", default_value="176.1,-195.1,2,0,0,180"),
            DeclareLaunchArgument(name="town", default_value="Town01"),
            DeclareLaunchArgument(name="passive", default_value="False"),
            DeclareLaunchArgument(name="synchronous_mode", default_value="False"),
            DeclareLaunchArgument(name="synchronous_mode_wait_for_vehicle_control_command", default_value="False"),
            DeclareLaunchArgument(name="fixed_delta_seconds", default_value="0.05"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("carla_ros_bridge"), "carla_ros_bridge.launch.py"])]
                ),
                launch_arguments={
                    "host": LaunchConfiguration("host"),
                    "port": LaunchConfiguration("port"),
                    "town": LaunchConfiguration("town"),
                    "timeout": LaunchConfiguration("timeout"),
                    "passive": LaunchConfiguration("passive"),
                    "synchronous_mode": LaunchConfiguration("synchronous_mode"),
                    "synchronous_mode_wait_for_vehicle_control_command": LaunchConfiguration(
                        "synchronous_mode_wait_for_vehicle_control_command"
                    ),
                    "fixed_delta_seconds": LaunchConfiguration("fixed_delta_seconds"),
                }.items(),
            ),
            ################################################
            ############ Vehicle & start point #############
            ################################################
            ############ option 1: custom start point
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("carla_spawn_objects"), "carla_spawn_objects.launch.py"])]
                ),
                launch_arguments={
                    "host": LaunchConfiguration("host"),
                    "port": LaunchConfiguration("port"),
                    "timeout": LaunchConfiguration("timeout"),
                    "vehicle_filter": LaunchConfiguration("vehicle_filter"),
                    "role_name": LaunchConfiguration("role_name"),
                    "spawn_point": LaunchConfiguration("spawn_point"),
                    "objects_definition_file": PathJoinSubstitution(
                        [FindPackageShare("testpkg"), "config/objects.json"]
                    ),
                    "spawn_point_ego_vehicle": LaunchConfiguration("spawn_point"),
                }.items(),
            ),
            ############ option 2: carla default start point
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [
            #             PathJoinSubstitution(
            #                 [FindPackageShare("carla_spawn_objects"), "carla_example_ego_vehicle.launch.py"]
            #             )
            #         ]
            #     ),
            #     launch_arguments={
            #         "host": LaunchConfiguration("host"),
            #         "port": LaunchConfiguration("port"),
            #         "timeout": LaunchConfiguration("timeout"),
            #         "vehicle_filter": LaunchConfiguration("vehicle_filter"),
            #         "role_name": LaunchConfiguration("role_name"),
            #         "spawn_point": LaunchConfiguration("spawn_point"),
            #     }.items(),
            # ),
            ################################################
            ############ Pygame window #####################
            ################################################
            ############ option 1: use custom pygame window
            Node(
                package="testpkg",
                executable="custom_manual_control",
                name=["custom_manual_control", LaunchConfiguration("role_name")],
                output="screen",
                emulate_tty=True,
                parameters=[{"role_name": LaunchConfiguration("role_name")}],
            ),
            ############ option 2: use default pygame window
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [PathJoinSubstitution([FindPackageShare("carla_manual_control"), "carla_manual_control.launch.py"])]
            #     ),
            #     launch_arguments={"role_name": LaunchConfiguration("role_name")}.items(),
            # ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
