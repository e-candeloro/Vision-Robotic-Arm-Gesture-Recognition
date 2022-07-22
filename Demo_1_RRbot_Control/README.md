# Vision Robotic Arm Control using ROS & Python OpenCV

This is the project branch that contains the ROS workspace to use for running all the scripts and simulations in a single machine with ROS Noetic and Gazebo installed. 

Follow the guide on the ROS folder README.md to install correctly all the required software.

## Packages
In the workspace, you can find the following packages:
- gazebo_ros_demos: for the rrbot simulation and control using Gazebo
- python_robot_pub: for the publisher class to use to publish data to a topic with python3
- vision_arm_control: that contains the main.py script that launchs a webcam and the pose estimation/hand aperture functions. It uses the python_robot_pub package to publish info to the 2 robot joints

## Setup and running the scripts

### 0. Install the python requirements
Go to the `ROS/python_example_ros/src/vision_arm_control/scripts` folder and open a new terminal there.
Install the python3 requirements:

    cd ROS/python_example_ros/src/vision_arm_control/scripts
    pip install -r requirements.txt

### 1. Build the workspace with catkin
    
    cd ROS/python_example_ros
    catkin init
    catkin_make

### 2. Launch the rrbot Gazebo simulation

    source devel/setup.bash
    roslaunch rrbot_gazebo rrbot_world.launch

### 3. Launch a roscore
In a new terminal, type:

    roscore
to launch a the master node (leave it running)

### 4. Launch the rrbot controllers
Open a new terminal and input:

    source devel/setup.bash
    roslaunch rrbot_control rrbot_control.launch

### 5. Launch the main script to control the rrbot
Open a new terminal and input:

    source devel/setup.bash
    rosrun vision_arm_control main.py