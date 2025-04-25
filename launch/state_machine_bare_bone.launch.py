from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # 'autorepeat_rate': 100.0,
            }],
    )

    state_machine_node = Node(
        package="uf850_pkg",
        executable="state_machine_node_bare_bone",
    )

    ld.add_action(joy_node)
    ld.add_action(state_machine_node)

    return ld