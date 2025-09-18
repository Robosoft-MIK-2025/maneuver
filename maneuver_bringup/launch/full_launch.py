from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    bridge_config = PathJoinSubstitution(
            [
                FindPackageShare("maneuver_bringup"),
                "config",
                "bridge.yaml",
            ]
        )
    



    return LaunchDescription([
        ExecuteProcess(
            cmd=['cd', '/home/mobile/PX4-Autopilot', '\n', 
                 'make', 'px4_sitl', 'gz_x500_depth'],
            output='log',
            shell=True
        ),
        TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
                    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                    output='log',
                    shell=True),
            ]
        ),
        TimerAction(
            period=15.0,
            actions=[ExecuteProcess(
                    cmd=['cd', '/home/mobile/Q_ground_control/', '\n', 
                         'APPIMAGE_EXTRACT_AND_RUN=1', './QGroundControl-x86_64.AppImage'],
                    output='log',
                    shell=True),
            ]
        ),
        TimerAction(
            period=25.0,
            actions=[
                Node(package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="depth_camera_bridge",
                    output="screen",
                    parameters=[bridge_config] 
                ),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0',  # Translation in x
                '--y', '0.0',  # Translation in y
                '--z', '0.0',  # Translation in z
                '--roll', '0.0',  # Roll angle in radians
                '--pitch', '0.0',  # Pitch angle in radians
                '--yaw', '0.0',  # Yaw angle in radians
                '--frame-id', 'world',  # Parent frame
                '--child-frame-id', 'map'  # Child frame
            ],
            name='world_to_map_tf',
            output="log",
        ),
    ])