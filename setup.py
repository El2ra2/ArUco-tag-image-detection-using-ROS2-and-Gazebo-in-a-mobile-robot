import os
from setuptools import find_packages, setup

package_name = 'looking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            ['launch/display.launch.py']),


        (os.path.join('share', package_name, 'world'),
            ['env/env.sdf']),

        (os.path.join('share', package_name, 'rviz'),
            ['rviz/config.rviz']),
        
        (os.path.join('share', package_name, 'config'),
            ['config/bridge_config.yaml']),

        (os.path.join('share', package_name, 'sdf'),
            ['robot_description/robot_description.sdf']),

        (os.path.join('share', package_name, 'world'),
            ['env/aruco_marker2.jpg']),
        (os.path.join('share', package_name, 'world'),
            ['env/aruco_marker3.png']),
        (os.path.join('share', package_name, 'world'),
            ['env/aruco_marker5.png']),
        (os.path.join('share', package_name, 'world'),
            ['env/aruco_marker9.png']),





    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vivek',
    maintainer_email='vivek@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = looking_robot.aruco_detector:main'
        ],
    },
)
