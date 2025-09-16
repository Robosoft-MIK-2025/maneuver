import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('maneuver_path_planner'),
        'path_planer.yaml'
    )

    urdf_path = os.path.join(
        get_package_share_directory('maneuver_bringup'),
        'description', 
        'robot.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_description = {'robot_description': robot_desc}

    robot_description_semantic = {'robot_description_semantic': '<?xml version="1.0" ?><robot name=""></robot>'}

    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[robot_description]
        # ),

        Node(
            package = 'moveit_ros_move_group',
            executable = 'move_group',
            name = 'rrt_star_node',
            parameters = [
                config,
                robot_description,
                robot_description_semantic
            ],
            output = 'screen',
            remappings=[('/move_group/result', '/global_path')]
        )
    ])

