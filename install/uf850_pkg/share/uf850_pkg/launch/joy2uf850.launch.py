from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    uf850_node = Node(
        package="uf850_pkg",
        executable="uf850_node",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # 'autorepeat_rate': 100.0,
            }],
    )

    joy2uf850_node = Node(
        package="uf850_pkg",
        executable="joy2uf850_node",
    )

    ld.add_action(uf850_node)
    ld.add_action(joy_node)
    ld.add_action(joy2uf850_node)

    return ld