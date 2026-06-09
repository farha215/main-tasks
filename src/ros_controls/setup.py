from setuptools import find_packages, setup
from glob import glob 
import os

package_name = 'ros_controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    

      data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line tells ROS to install all files in the launch folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        (os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.onnx'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cupcake',
    maintainer_email='cupcake@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop2 =ros_controls.teleop2:main',
            'detection =ros_controls.obj_detect:main',
            'viewer =ros_controls.image_viewer:main',
            'viewer_3=ros_controls.viewer_3:main',
            'detection_3=ros_controls.obj_detect_3:main',
            'simple_cam=ros_controls.simple_cam:main',
            'pyz_detect=ros_controls.pyzed_detect:main',
            'hsv_detect=ros_controls.hsv_detector:main',
            'combined_detect=ros_controls.combined_2:main',
            'combined_detect_gpu=ros_controls.combined_3:main',
            'combined_detect_gpu_2=ros_controls.combined_4:main',
            #'dataset_collector =ros_controls:dataset_collector:main',
        ],
    },
)
