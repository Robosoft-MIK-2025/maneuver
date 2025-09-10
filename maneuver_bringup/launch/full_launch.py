from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['cd', '/home/mobile/PX4-Autopilot', '\n', 'make', 'px4_sitl', 'gz_x500_depth'],
            output='screen',
            shell=True
        ),
        TimerAction(
            period=15.0,
            actions=[ExecuteProcess(
                    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                    output='log',
                    shell=True),
            ]
        ),
        TimerAction(
            period=15.0,
            actions=[ExecuteProcess(
                    cmd=['cd', '/home/mobile/Q_ground_control/', '\n', 'mobile', 'su', '\n', 'APPIMAGE_EXTRACT_AND_RUN=1', './QGroundControl-x86_64.AppImage'],
                    output='log',
                    shell=True),
            ]
        ),
    ])