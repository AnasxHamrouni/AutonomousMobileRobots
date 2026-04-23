from setuptools import setup

package_name = 'bug_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Bug algorithm navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug0_planner_node = bug_navigation.bug0_planner:main',
            'bug1_planner_node = bug_navigation.bug1_planner:main',
            'a_star_planner_node = bug_navigation.a_star_planner:main',
        ],
    },
)
