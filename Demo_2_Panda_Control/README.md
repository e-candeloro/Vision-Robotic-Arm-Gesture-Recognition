# Vision Robotic Arm Control using ROS & Python OpenCV

This is the project branch that contains the ROS workspace to use for running the Panda Robot control script and simulation with vision control.

## Packages
In the workspace, you can find the following set of packages:
- panda simulator: set of packages from [this repository](https://github.com/justagist/panda_simulator) to enable the control of the Panda Franka Emika robot trough ROS topics and the Gazebo simulation
- vision_arm_control: that contains the main.py that launchs a vision script for the pose + hand features extraction and ROS Pub/Sub to control the Panda robot.

# ROS Setup Instructions

## Installing Ubuntu Focal Fossa on a VM
1. Download Ubuntu Focal Fossa from [here](https://releases.ubuntu.com/20.04/)
2. Install Ubuntu on a Virtual Machine ([guide in italian for Virtual Box](https://www.youtube.com/watch?v=62-hJarauK4&list=PLhlcRDRHVUzR-5TKDC1VPMtyhEyyQ5uwy)).
Allocate at least 25 GB of disk space and install with the minimal installation option.
3. Update Ubuntu `sudo apt-get update && sudo apt-get upgrade`.
4. Tweak VM settings to [improve performances](http://www.rawinfopages.com/tips/2017/07/speed-up-virtualbox-in-windows/).

## Installing ROS 1 Noetic
### 1. Configure Ubuntu repo
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the [Ubuntu guide for instructions on doing this](https://help.ubuntu.com/community/Repositories/Ubuntu).

Open a terminal and imput those commands:
    
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

### 2. Set up keys
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
### 3. Installation of the full version of ROS Noetic (recommended)
    sudo apt update
    sudo apt install ros-noetic-desktop-full
### 4. Env. Setup
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
This will ensure that every time you open a shell the command `source /opt/ros/noetic/setup.bash` is executed automatically.
### 5. Install dependencies for building packages
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
    #initialize rosdep
    sudo rosdep init
    rosdep update

    #install catkin
    sudo apt install python3-catkin-tools python3-osrf-pycommon
    
### 6. Install packages to control robots

    #install ros controllers
    sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers

## Useful Links
Refer to [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu) for additional info.
Also this [video tutorial for ROS installation](https://youtu.be/PowY8dV36DY).

- [ROS Wiki](http://wiki.ros.org/Documentation)
- [ROS Beginner Python Tutorial (YT Playlist)](https://www.youtube.com/playlist?list=PLAjUtIp46jDcQb-MgFLpGqskm9iB5xfoP)
- [Project example + video tutorial](https://github.com/utra-robosoccer/Tutorials-2020)

# Project Setup Instructions

### 0. Install the python requirements
Go to the `ROS\panda_control\src\vision_arm_control\scripts` folder and open a new terminal there.
Install the python3 requirements:

    pip install -r requirements.txt

### 1. Install the Panda Simulator packages
    
    cd ROS/panda_control/src
    git clone -b noetic-devel https://github.com/justagist/panda_simulator

Install the package using the in-built script

    cd panda_simulator
    sh ./build_ws.sh

Follow the procedure and wait for the script to finish. It will automatically launch the `catkin build` command, so it may take a while to finish.
If the script is not successfull try those commands:

    catkin clean
    rm -r .catkin_tool
    
Then

    catkin build
 
### 2. Launch the Gazebo Panda simulation

Inside the folder `~/panda_control/` open a terminal and execute these commands:

    source devel/setup.bash
    roslaunch rrbot_gazebo rrbot_world.launch

If everything is setup correctly, Gazebo will launch showing the panda robot in a folded starting position. If any errors occurs, try cleaning and rebuilding the workspace:

    catkin clean
    catkin build

### 4. Start the vision script
You need at least one webcam/camera in your pc to run this script. By default, the first system camera is used (source = 0). Open a new terminal and type:
    
    source devel/setup.bash
    rosrun vision_arm_control main.py

After some initial configurations, a windows showing the webcam/camera image will appear. It is now possible to control the Panda Robot with gestures.

To stop the script, press !q" on the keyboard. To stop the Gazebo simulation, go to the terminal that is executing the program and press CTRL + C