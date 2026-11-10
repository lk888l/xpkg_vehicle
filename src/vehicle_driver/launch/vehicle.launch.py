from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("vehicle_driver"), "config", "vehicle.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("imu_can_interface", default_value="can1"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("auto_initialize", default_value="true"),
        Node(
            package="vehicle_driver", executable="imu_node", name="imu",
            namespace=LaunchConfiguration("namespace"), output="screen",
            condition=IfCondition(LaunchConfiguration("start_imu")),
            parameters=[LaunchConfiguration("config"), {
                "imu_can_interface": LaunchConfiguration("imu_can_interface"),
            }],
        ),
        Node(
            package="vehicle_driver", executable="vehicle_node", name="vehicle",
            namespace=LaunchConfiguration("namespace"), output="screen",
            parameters=[LaunchConfiguration("config"), {
                "can_interface": LaunchConfiguration("can_interface"),
                "auto_initialize": ParameterValue(
                    LaunchConfiguration("auto_initialize"), value_type=bool),
            }],
        ),
    ])
