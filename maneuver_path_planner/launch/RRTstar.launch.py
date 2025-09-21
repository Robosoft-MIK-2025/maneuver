import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('maneuver_bringup'),
        'urdf',
        'drone.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {'robot_description': robot_desc}

    srdf_path = os.path.join(
        get_package_share_directory('drone_moveit_config'),
        'config',
        'drone.srdf'
    )
    with open(srdf_path, 'r') as infp:
        robot_desc_semantic = infp.read()
    robot_description_semantic = {'robot_description_semantic': robot_desc_semantic}

    kinematics_path = os.path.join(
        get_package_share_directory('drone_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    ompl_planning = os.path.join(
        get_package_share_directory('drone_moveit_config'),
        'config',
        'ompl_planning.yaml'
    )

    moveit_controllers = os.path.join(
        get_package_share_directory('drone_moveit_config'),
        'config',
        'moveit_controllers.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('maneuver_path_planner'),
        'rviz',
        'moveit_config.rviz'
    )

    sensors_yaml = os.path.join(
    get_package_share_directory('drone_moveit_config'),
    'config',
    'sensors.yaml'
    )

    return LaunchDescription([
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[robot_description]
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning,         
                kinematics_path,         
                moveit_controllers,
                sensors_yaml,   
                {
                    "planning_pipelines": ["ompl"],
                    "default_planning_pipeline": "ompl",
                    "ompl": {
                        "planning_plugin": "ompl_interface/OMPLPlanner",
                        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                            "default_planner_request_adapters/FixWorkspaceBounds "
                                            "default_planner_request_adapters/FixStartStateBounds "
                                            "default_planner_request_adapters/FixStartStateCollision "
                                            "default_planner_request_adapters/FixStartStatePathConstraints",
                        "start_state_max_bounds_error": 0.1
                    },
                    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                    "moveit_simple_controller_manager": {
                        "controller_names": ["drone_controller"]
                    }
                }
            ],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            name="static_tf_map_to_base"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=[robot_description, robot_description_semantic]
        ),

        Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            output="screen",
            parameters=[{
                'frame_id': 'map',
                'resolution': 0.05
            }]
        )
    ])
