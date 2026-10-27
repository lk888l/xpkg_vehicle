from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    stamped = ParameterValue(LaunchConfiguration("use_stamped_cmd_vel"), value_type=bool)
    driver = Node(
        package="vehicle_driver", executable="vehicle_node", name="vehicle", namespace=namespace,
        output="screen", condition=IfCondition(LaunchConfiguration("start_driver")),
        parameters=[LaunchConfiguration("config"), {
            "can_interface": LaunchConfiguration("can_interface"),
            "auto_initialize": ParameterValue(LaunchConfiguration("auto_initialize"), value_type=bool),
            "use_stamped_cmd_vel": stamped,
        }],
    )
    gui = Node(package="vehicle_gui", executable="vehicle_gui", name="vehicle_gui", namespace=namespace,
               output="screen", parameters=[{"use_stamped_cmd_vel": stamped}],
               arguments=["--quit-after", LaunchConfiguration("quit_after")])
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("quit_after", default_value="0", description="Close GUI after N seconds; 0 keeps it open"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("auto_initialize", default_value="true"),
        DeclareLaunchArgument("use_stamped_cmd_vel", default_value="true"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([
            FindPackageShare("vehicle_driver"), "config", "vehicle.yaml"])),
        RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=[
            EmitEvent(event=Shutdown(reason="Vehicle console closed"))])),
        driver, gui,
    ])
