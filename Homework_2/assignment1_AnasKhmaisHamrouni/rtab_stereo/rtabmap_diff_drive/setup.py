import os
from glob import glob
from setuptools import setup

package_name = 'rtabmap_diff_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='RTAB-Map homework with stereo camera on a differential drive robot in Gazebo (ROS 2 Humble)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
