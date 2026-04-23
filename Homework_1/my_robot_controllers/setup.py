from setuptools import find_packages, setup

package_name = 'my_robot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ghadeer',
    maintainer_email='ghadeer@todo.todo',
    description='ROS 2 controllers package with pure pursuit, P, Stanley, and MPC nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = my_robot_controllers.pure_pursuit_controller:main',
            'p_controller = my_robot_controllers.p_controller:main',
            'stanley_controller = my_robot_controllers.stanley_controller:main',
            'mpc_controller = my_robot_controllers.mpc_controller:main',
        ],
    },
)
