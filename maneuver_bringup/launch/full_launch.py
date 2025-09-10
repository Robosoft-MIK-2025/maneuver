from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['cd', '/home/mobile/PX4-Autopilot', '\n', 'make', 'px4_sitl', 'gz_x500_depth'],
            output='log',
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
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #     ]
        # ),
    ])