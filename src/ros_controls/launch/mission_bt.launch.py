import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory('main_task'), 'config', 'mission_params.yaml'
    )
    
    maintask_bt = Node(
        package='main_task',
        executable='main_task',
        parameters=[mission_params],
        output='screen'
    )

    groot2 = ExecuteProcess(
        cmd=['/home/farha/Downloads/Groot2-v1.9.0-x86_64.AppImage'],
        output='screen'
    )

    return LaunchDescription([
        maintask_bt,
        groot2
    ])
