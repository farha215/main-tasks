import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node  # <-- Added Node here
from ament_index_python.packages import get_package_share_directory

import glob


def generate_launch_description():
    
    detection_node_3 = Node(
    package='ros_controls',
    executable='detection_3',
    output='screen',
    parameters=[{
        'input_objects_topic': '/zed_front/zed2i_front/zed_node/obj_det/objects',
        'input_image_topic':   '/zed_front/zed2i_front/zed_node/left/image_rect_color',
        'publish_image': True,
    }],
)
    # Node 2: CV2 Image Viewer 
    viewer_node = Node(
        package='ros_controls',
        executable='viewer',  # Name matching your setup.py/CMakeLists Entry
        name='zed_image_viewer',
        output='screen'
    )

    viewer_node_3 = Node(
    package='ros_controls',
    executable='viewer_3',  # Must match the entry point string in your setup.py or CMakeLists.txt
    name='zed_image_viewer',
    output='screen',
    parameters=[{
        'input_image_topic': '/detection_image'
    }],
    )

    pyz_detect = Node(
    package='ros_controls',
    executable='pyz_detect',   # update setup.py entry point too
    output='screen',
    parameters=[{
        'serial_number': 38605411,   # your front cam serial
        'publish_image': True,
        'confidence_threshold': 50.0,
        'detection_model': 'MULTI_CLASS_BOX_FAST',
    }],

)

    # Return everything to the launch description engine
    return LaunchDescription([
        #cam_front, 
        #cam_down,
        #detection_node,
        detection_node_3,
        #pyz_detect,
        viewer_node_3,
    ])