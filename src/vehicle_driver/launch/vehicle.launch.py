from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("auto_initialize", default_value="true"),
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
