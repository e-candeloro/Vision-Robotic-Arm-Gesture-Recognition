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

## Using the ROS packages in this repository
### 1. Clone this repository
Clone this repo using `git clone https://github.com/e-candeloro/Vision-Robotic-Arm-Control.git`.
### 2. Build the workspace with catkin
Move inside the folder `~/python_example_ros/`.
If you are in the repo main folder:
    
    cd ROS/python_example_ros
Then use catkin:

    catkin_make

After the following command, two new folders named "build" and "devel" should appear in the "python_example_ros" directory.

### 3. Initialize roscore and source devel setup
Every time you open a new terminal and  want to work with launch files and packages inside the workspace, you need to execute the command: `source devel/setup.bash`.

Now, every time you open a terminal, this command is executed.

### 4. Initialize a roscore
Open a new terminal, and type:

    roscore

Leave this terminal running roscore till the end of your simulation.
You can kill it with CTRL + C when you need to stop it.
When executing ros nodes, one and only one roscore needs to be running!

### 5. Open the package you want to use or .roslaunch file you need

In our case, with the Gazebo robot and publishers we want to do the following:

1. Open rrbot on gazebo:

        roslaunch rrbot_gazebo rrbot_world.launch
    If everything goes well, you should now have an open Gazebo simulation with the rrbot visible. Leave this terminal running!

2. Launch the rrbot controllers (on new terminal!)

        roslaunch rrbot_control rrbot_control.launch
    You should see no error messages.

3. Launch the publisher_rrbot.py (on yet another terminal!)
        
        rosrun python_pub_sub publisher_rrbot.py
    You should see some info printing on the terminal and the robot on Gazebo moving.
    Done! 🎉

In other cases, follow the same procedure but for other .roslaunch or nodes in other packages.


## Creating FROM SCRATCH a ROS Project/Workspace with a Package  using two Python scripts for pub-sub
Watch the first 5 videos of [this playlist](https://youtube.com/playlist?list=PLAjUtIp46jDcQb-MgFLpGqskm9iB5xfoP) for a step by step tutorial.

We need to setup the ROS workspace using catkin and then we can start to add ROS packages to the workspace/project.
We can think of a ROS workspace like the trunk of a tree and of ROS packages as the tree branches.
Every ROS project contains at least one package.
A ROS package can contain libraries, datasets, config files but most importantly a ROS node.

### 1. Creating a ROS Project
    #create a directory with a src sub-directory

    mkdir -p my_ros_workspace/src
    cd my_ros_workspace/

    #this will create two more folders "devel" and "build" to make a workspace

    catkin_make
### 2. Create a ROS Package
    cd src/
    catkin_create_pkg my_ros_package [dependecies here (optional)]

In our case, with the pub-sub example, we write:
    
    catkin_create_pkg my_ros_package rospy std_msgs

### 3. Exploring the package.xml and CMakeLists.txt files
These two files, located in `my_ros_workspace/src/my_ros_package`, describe the configurations and the dependencies of the package.

### 4. Adding Python scripts in a package
Create a `scripts` folder inside the `my_ros_package` directory:
    
    cd my_ros_package/
    mkdir scripts
Then create two python files and make them executable:
    
    cd scripts
    touch publisher_node.py
    touch subscriber_node.py
    chmod +x *
Open the CMakeLists.txt in the `my_ros_package` folder and, after the part
    
    catkin_package( 
    #  INCLUDE_DIRS include
    #  LIBRARIES python_pub_sub
    #  CATKIN_DEPENDS rospy std_msgs
    #  DEPENDS system_lib
    )
write:

    catkin_install_python(PROGRAMS scripts/publisher_node.py scripts/subscriber_node.py
    DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
    )
and save.


### 5. Writing and running the pub-sub package
You can now work on the python pub-sub scripts.
You can copy them from this repo, in the `python_example/python_pub_sub/scripts` folder.

- For the publisher, follow [this video tutorial](https://youtu.be/yqYvMEYJoTk).
- For the subscriber, follow [this video tutorial](https://youtu.be/MkiWj4VwZjc).

For running the scripts, follow the procedures in in the previous sections, using `roscore`, `source devel/setup.bash`, `roslaunch ros_package_name roslaunchfile.roslauch`, `rosrun ros_package_name rosnode.py`.

## Setting up a robot with controllers in Gazebo + ROS
Follow those steps:

1. Ensure you have installed ros noetic full and type `gazebo` in a terminal to see if you have gazebo installed
2. If you have gazebo installed, execute the following commands:
    
        sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

3. More info in this [tutorial](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
4. Read [this tutorial](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) to get a copy of a working robot in Gazebo
5. Read [this tutorial](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros) to learn how ROS Control works with Gazebo 
