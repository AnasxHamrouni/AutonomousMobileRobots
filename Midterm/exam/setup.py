from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'exam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghadeer',
    maintainer_email='ghadeer@todo.todo',
    description='AMR Midterm Exam - Ackermann steering robot with controller tasks',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_p_controller = exam.ackermann_p_controller:main',
            'ackermann_pure_pursuit = exam.ackermann_pure_pursuit:main',
            'ackermann_stanley = exam.ackermann_stanley:main',
            'obstacle_avoidance = exam.obstacle_avoidance:main',
            'ackermann_drive_node = exam.ackermann_drive_node:main',
        ],
    },
)
