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
        'urdf', 
        'drone.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {'robot_description': robot_desc}
    
     # SRDF — замените на путь к вашему SRDF, сгенерированному через MoveIt Setup Assistant
    srdf_path = os.path.join(
        get_package_share_directory('drone_moveit_config'),  # пакет, где хранится SRDF
        'config',
        'drone.srdf'
    )
    with open(srdf_path, 'r') as infp:
        robot_desc_semantic = infp.read()
    robot_description_semantic = {'robot_description_semantic': robot_desc_semantic}
    
    # Здесь добавляем kinematics.yaml
    kinematics_path = os.path.join(
        get_package_share_directory('drone_moveit_config'),
        'config',
        'kinematics.yaml'
    )
    # Загружаем как параметр
    kinematics_parameters = {'kinematics_yaml': kinematics_path}


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
            package = 'moveit_ros_move_group',
            executable = 'move_group',
            name = 'rrt_star_node',
            parameters = [
                config,
                robot_description,
                robot_description_semantic
            ],
            output = 'screen'
        ),
        
        Node(
	    package='tf2_ros',
	    executable='static_transform_publisher',
	    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
	    name='static_tf_map_to_base'
	),

        
        Node(
	    package="rviz2",
	    executable="rviz2",
	    name="rviz2",
	    output="screen",
	    arguments=["-d", os.path.join(
		get_package_share_directory('maneuver_path_planner'),
		'rviz',
		'moveit_config.rviz'
	    )],
	    parameters=[robot_description, robot_description_semantic]
	)


    ])

