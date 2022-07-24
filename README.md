# Vision Robotic Arm Gesture Recognition

## What is this project about?

**This repository allows the control of a 7 DOF robot (Panda by Franka Emika) using a webcam, with the movement of the operator body.**

### Demo

https://user-images.githubusercontent.com/67196406/180643477-1fb39a60-c666-49cc-a550-87cd11b2aa3e.mp4

## Why this project?

This is a work done for the course of [Smart Robotics](https://offertaformativa.unimore.it/corso/insegnamento?cds_cod=20-262&aa_ord_id=2009&pds_cod=20-262-2&aa_off_id=2021&lang=ita&ad_cod=IIM-64&aa_corso=2&fac_id=10005&coorte=2020&anno_corrente=2022&durata=2) taught at [University of Modena and Reggio Emilia (UNIMORE)](https://international.unimore.it/) in the second semester of the academic year 2021/2022.

This project aims to demostrate a **proof of concept** using both computer vision algorithms and robot simulation and control, combining **Python**,**OpenCV**, **Mediapipe** and **ROS + Gazebo**.

Controlling remotely a robot can be useful in dangerous environments, for medical assistance and for simulation/testing.

### Project Team

- Benedetta Fabrizi ([LinkedIn](https://www.linkedin.com/in/benedetta-fabrizi-54b7971b0) - [GitHub](https://github.com/BerniRubble))
- Emanuele Bianchi ([LinkedIn](https://www.linkedin.com/in/emanuele-bianchi240497/) - [GitHub](https://github.com/Manu2497))
- Ettore Candeloro ([LinkedIn](https://www.linkedin.com/in/ettore-candeloro-900081162/) - [GitHub](https://github.com/e-candeloro))
- Gabriele Rosati ([LinkedIn](https://www.linkedin.com/in/gabriele-rosati-4817b01a7/) - [GitHub](https://github.com/gabri1997))

## How this repository is organized

In the Demo_1_RRbot_Control we implemented a first version of our project with a simpler 2 DOF robot.

In the Demo_2_Panda_Control we implemented a second version of our project with a more complex robot and control with both the hand aperture and pose angles.

In every folder, there are README files that contains all the instruction to follow for installing ROS and setting up correctly the workspaces.

In the project_presentation folder, a Power Point file containing a brief description of the overall work is present.

In the images and videos folders all the media used for this repository and the ppt presentation can be found.

## High Level Architecture

![Project Architecture](https://github.com/e-candeloro/Vision-Robotic-Arm-Gesture-Recognition/blob/master/Images/ROS%20%2B%20Robots/High%20level%20final%20architecture.png)

1. The video stream of the Operator is taken with a webcam/camera
2. Each frame is passed to the vision_arm_control of the ROS workspace.
3. The vision script detects the hand and pose keypoints and then compute the hand aperture and pose angles
4. The hand aperture and angles are sent to the Panda robot in the Gazebo simulation via a Pub/Sub ROS node
5. The robot is simulated and controlled almost real-time thanks to the Panda Simulator Interfaces packages.


