from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    xnode_template = Node(
        name="xnode_template",
        package="xpkg_template",
        executable="xnode_template",
        output="screen",
        parameters=[{
            "out_str": "hello",
        }]
    )
    
    launch_description = LaunchDescription([xnode_template])
    
    return launch_description

