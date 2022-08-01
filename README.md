# Vision Robotic Arm Gesture Recognition

## What is this project about?

**This repository contains a ROS workspace for the control of a 7 DOF robot (Panda by Franka Emika) or a 2 DOF robot (RRbot) using a webcam, with the movement of the operator body.**

### Demo Panda

https://user-images.githubusercontent.com/67196406/180643477-1fb39a60-c666-49cc-a550-87cd11b2aa3e.mp4

### Demo RRbot

https://user-images.githubusercontent.com/67196406/180643684-1788eaad-869d-4e96-8824-975ff3582e98.mp4

## Why this project?

This is a work done for the course of [Smart Robotics](https://offertaformativa.unimore.it/corso/insegnamento?cds_cod=20-262&aa_ord_id=2009&pds_cod=20-262-2&aa_off_id=2021&lang=ita&ad_cod=IIM-64&aa_corso=2&fac_id=10005&coorte=2020&anno_corrente=2022&durata=2) taught at [University of Modena and Reggio Emilia (UNIMORE)](https://international.unimore.it/) in the second semester of the academic year 2021/2022.

This project aims to demostrate a **proof of concept** using both computer vision algorithms and robot simulation and control, combining **Python**,**OpenCV**, **Mediapipe** and **ROS + Gazebo**.

In fact, controlling remotely a robot can be useful in dangerous environments, for medical assistance and for simulation/testing.
Vision control for anthropomorphic robots can be a new way to work with remote operations in a simpler manner.

### Project Team

- Benedetta Fabrizi ([LinkedIn](https://www.linkedin.com/in/benedetta-fabrizi-54b7971b0) - [GitHub](https://github.com/BerniRubble))
- Emanuele Bianchi ([LinkedIn](https://www.linkedin.com/in/emanuele-bianchi240497/) - [GitHub](https://github.com/Manu2497))
- Ettore Candeloro ([LinkedIn](https://www.linkedin.com/in/ettore-candeloro-900081162/) - [GitHub](https://github.com/e-candeloro))
- Gabriele Rosati ([LinkedIn](https://www.linkedin.com/in/gabriele-rosati-4817b01a7/) - [GitHub](https://github.com/gabri1997))

## How this repository is organized

**NOTE**: In every folder, there are README.md files that contains all the instruction to follow for installing ROS and setting up correctly the workspaces.

- In the **Demo_1_RRbot_Control** we implemented a first version of our project with a simpler 2 DOF robot.
- In the **Demo_2_Panda_Control** we implemented a second version of our project with a more complex robot and control with both the hand aperture and pose angles.
- In the **project_presentation folder**, a Power Point file containing a brief description of the overall work is present.
- In the **images** and **videos** folders all the media used for this repository and the ppt presentation can be found.

## High Level Architecture

![Project Architecture](https://github.com/e-candeloro/Vision-Robotic-Arm-Gesture-Recognition/blob/master/Images/ROS%20%2B%20Robots/High%20level%20final%20architecture.png)

1. The video stream of the Operator is taken with a webcam/camera
2. Each frame is passed to the vision_arm_control of the ROS workspace.
3. The vision script detects the hand and pose keypoints and then compute the hand aperture and pose angles
4. The hand aperture and angles are sent to the Panda robot in the Gazebo simulation via a Pub/Sub ROS node
5. The robot is simulated and controlled almost real-time thanks to the Panda Simulator Interfaces packages.

## Vision Algorithm and Package

For the vision part of the project, we used the OpenCV and Mediapipe libraries.
We created a **HandDetector** and **PoseDetector** modules to extract the hand aperture and pose angles from a video stream.
In the **main.py** script we capture the video footage and also use a pub/sub to interact with the robot.

### **Hand Detector and computing the hand aperture**
The hand detector class is composed of the following methods:

![hand_det_class](Images/Vision/HandDetector.png)


### Hand detector demo
<img src="videos/Vision/GIFs/Hand%20Aperture%20Demo.gif" width="700">

### **Hand Aperture pipeline**

1. The four hand index tips are located, along the base of the hand (middle point between wrist and thumb base)
2. The median point between the fingers tips (thumb excluded) is computed
3. The L2 distance between the base of the hand point and the median fingers position is computed.
4. The distance is then normalized by the palm size and remapped between the value 0 and 100


### **Pose Detector and computing the pose angles**
The hand detector class is composed of the following methods:

![pose_det_class](Images/Vision/Pose%20Detector.png)

### Pose detector demo

<img src="videos/Vision/GIFs/pose_showcase.gif" height="200"> <img src="videos/Vision/GIFs/Angleshowcase.gif" height="200"> 

### **Pose Angles pipeline**
To extract the angles between a set of 3 keypoints (like the elbow angle given the points 12, 14, 16) we make use of the property of the scalar product between two vectors.

1. We create two vectors with the same origin (at the central keypoint, where we want to know the angle value)
2. We use the following formula to compute the angle between those two vectors


<img src="Images\Vision\Angle formula.png" width="200">

3. With this method we can consider both the 2D and 3D keypoints for more robustness!


## ROS and Robot Simulation Package

For the robot simulation and control part of the project, we used the ROS with Gazebo.
We created ad hoc ROS nodes with a publisher and subsriber to various topics to control the RRbot while we leveraged the [Panda Simulator Package](https://github.com/justagist/panda_simulator) for the Panda arm control.

Both the two demos make use of a pub/sub mechanism but in case of the Panda Control package, the publishing is done with a list of joint values.

### Panda Arm control
Focusing on the second and final demo of this project,for moving our robot we use:

- One hand for the sixth joint
- Right-elbow angle for the fourth joint
- Left-elbow angle for the second joint
- Left-shoulder angle for the first joint

The other 3 joints are fixed 


<img src="Images/ROS%20%2B%20Robots/Joint%20mapping%20control%20human.jpg" height="300">
<img src="Images/ROS%20%2B%20Robots/Joint%20mapping%20control%20robot.jpg" height="300">


## Future Improvements

- **Gestures complexity and usability**: Increase the set of possible gestures by adding different types of movements and improving the usability.

- **Test the prototype on a high-end machine**, then transpose it over a real robot.

- **Divide each node into a standalone machine**, which will be connected to the master nodes remotely.

- **Improve the connectivity of the solution**, by exploiting camera and devices that could be connected remotely.





