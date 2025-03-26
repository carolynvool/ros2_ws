from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autolab',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node=my_robot_controller.my_first_node:main", # attaching to the main function of the folder
            # follow this for each new script
            'draw_circle = my_robot_controller.draw_circle:main',
            'pose_sub = my_robot_controller.pose_subscriber:main',
            'turtle_controller = my_robot_controller.turtle_controller:main',
            "mapping = my_robot_controller.mapping:main",
        ],
    },
)
