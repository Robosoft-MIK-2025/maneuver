import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    global_planner_param = os.path.join(
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

        container = ComposableNodeContainer(
            name="hybrid_planning_container",
            namespace="/",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                    name="global_planner",
                    parameters=[
                        global_planner_param,
                        robot_description,
                        robot_description_semantic,
                    ],
                ),
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::LocalPlannerComponent",
                    name="local_planner",
                    parameters=[
                        local_planner_param,
                        robot_description,
                        robot_description_semantic,
                    ],
                ),
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::HybridPlanningManager",
                    name="hybrid_planning_manager",
                    parameters=[hybrid_planning_manager_param],
                ),
            ],
            output="screen",
        )


        # Node(
        #     package = 'moveit_ros_move_group',
        #     executable = 'move_group',
        #     name = 'rrt_star_node',
        #     parameters = [
        #         config,
        #         robot_description,
        #         robot_description_semantic
        #     ],
        #     output = 'screen',
        #     remappings=[('/move_group/result', '/global_path')]
        # )
    ])

