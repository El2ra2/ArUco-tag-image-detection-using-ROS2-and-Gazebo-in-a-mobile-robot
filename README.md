# ArUco tag image detection using ROS2 and Gazebo in a mobile robot
Simulated a camera-equipped mobile robot in Gazebo and detected ArUco markers in the environment using ROS2.

## Pre-requisites:
This project is done in ROS2 Jazzy and Gazebo Harmonic, changes might need to be made for previous versions. Additionally, xterm needs to be installed (command: sudo apt install xterm) to control the robot using keyboard inputs. Make sure colcon is installed (command: sudo apt install python3-colcon-common-extensions)

## How to run:
- After downloading the files, run (command: colcon build) inside the directory 'open_robot'. 

- Then source the directory using (command: source ~/open_robot/install/setup.bash), or paste the command in your bash file '.bashrc' and run (source ~/.bashrc). 

- Use the launch file (command: ros2 launch looking_robot display.launch.py) to bring up the simulation, rviz, and the detection node. 

- Make sure to click inside the keyboard input terminal to move the robot, the instructions to move would be shown in the terminal. Also make sure the marker is completely inside the area detected by the camera. The Marker ID and its position and orientation would be displayed in the terminal.
