#!/usr/bin/env python3

# import libraries and HandDetector + PoseDetector modules
import time

import rospy
import cv2

from copy import deepcopy
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand, RobotState
from Detector_Modules.HandDetectorModule import HandDetector
from Detector_Modules.PoseDetectorModule import PoseDetector

# initialize global variables
t = 0
coefficient = 0.3
coefficient_body = 1.5
vals = []
vels = []
names = ['panda_joint1', 'panda_joint2', 'panda_joint3',
         'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

neutral_pose = [-0.017792060227770554,
                -0.7601235411041661,
                0.019782607023391807,
                -2.342050140544315,
                0.029840531355804868,
                1.5411935298621688,
                0.7534486589746342]

starting_pose = [0.04941637656952391,
                 -0.6935505764316097,
                 0.0835676500556497,
                 -2.2765016376491585,
                 -0.03950194350709957,
                 1.6093278345679556,
                 0.7534486589746342]

# callback function for the robot state


def callback(msg):

    global vals, vels
    temp_vals = []
    temp_vels = []
    for n in names:
        idx = msg.name.index(n)
        temp_vals.append(msg.position[idx])
        temp_vels.append(msg.velocity[idx])

    vals = deepcopy(temp_vals)
    vels = deepcopy(temp_vels)


def state_callback(msg):
    global t
    if t % 100 == 0:
        t = 1
        rospy.loginfo("============= Current robot state: ============\n")
        rospy.loginfo("Cartesian vel: \n{}\n".format(msg.O_dP_EE))
        rospy.loginfo(
            "Gravity compensation torques: \n{}\n".format(msg.gravity))
        rospy.loginfo("Coriolis: \n{}\n".format(msg.coriolis))
        rospy.loginfo("Inertia matrix: \n{}\n".format(msg.mass_matrix))
        rospy.loginfo("Zero Jacobian: \n{}\n".format(msg.O_Jac_EE))

        rospy.loginfo("\n\n========\n\n")

    t += 1

# method for sending the robot to a neutral pose


def send_to_neutral():
    """
        DON'T USE THIS ON REAL ROBOT!!! 

    """
    temp_pub = rospy.Publisher(
        '/panda_simulator/motion_controller/arm/joint_commands', JointCommand, queue_size=1, tcp_nodelay=True)
    # Create JointCommand message to publish commands
    pubmsg = JointCommand()
    pubmsg.names = names  # names of joints (has to be 7)
    # JointCommand msg has other fields (velocities, efforts) for
    pubmsg.position = neutral_pose
    # when controlling in other control mode
    # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)
    pubmsg.mode = pubmsg.POSITION_MODE
    curr_val = deepcopy(vals)

    while all(abs(neutral_pose[i]-curr_val[i]) > 0.01 for i in range(len(curr_val))):
        temp_pub.publish(pubmsg)
        curr_val = deepcopy(vals)


def main(fps_cap=60, show_fps=True, source=0):
    """
    Capture webcam video from the specified "source" (default is 0) using the opencv VideoCapture function.
    It's possible to cap/limit the number of FPS using the "fps_cap" variable (default is 60) and to show the actual FPS footage (shown by default).
    The program stops if "q" is pressed or there is an error in opening/using the capture source.

    :param: fps_cap (int)
        max framerate allowed (default is 60)
    :param: show_fps (bool)
        shows a real-time framerate indicator (default is True)
    :param: source (int)
        select the webcam source number used in OpenCV (default is 0)
    """
    assert fps_cap >= 1, f"fps_cap should be at least 1\n"
    assert source >= 0, f"source needs to be greater or equal than 0\n"

    # instantiation of the HandDetector and PoseDetector
    HandDet = HandDetector()
    PoseDet = PoseDetector(detCon=0.7, trackCon=0.7, modCompl=1)

    cv2.setUseOptimized(True)

    ctime = 0  # current time (used to compute FPS)
    ptime = 0  # past time (used to compute FPS)
    prev_time = 0  # previous time variable, used to set the FPS limit

    fps_lim = fps_cap  # FPS upper limit value, needed for estimating the time for each frame and increasing performances

    time_lim = 1. / fps_lim  # time window for each frame taken by the webcam

    # initializing the test node to publish to the robot
    rospy.init_node("test_node")
    rospy.wait_for_service('/controller_manager/list_controllers')
    rospy.loginfo("Starting node...")

    rospy.sleep(5)

    # initializing the publisher object to the topic /panda_simulator/motion_controller/arm/joint_commands
    pub = rospy.Publisher('/panda_simulator/motion_controller/arm/joint_commands',
                          JointCommand, queue_size=1, tcp_nodelay=True)

    # Subscriber to robot joint state
    rospy.Subscriber(
        '/panda_simulator/custom_franka_state_controller/joint_states', JointState, callback)

    # Subscribe to robot state (Refer JointState.msg to find all available data.
    # Note: All msg fields are not populated when using the simulated environment)
    rospy.Subscriber(
        '/panda_simulator/custom_franka_state_controller/robot_state', RobotState, state_callback)

    # set the pub/sub rate to 1000 Hz
    rate = rospy.Rate(1000)

    rospy.loginfo("Not received current joint state msg")
    # setting the robot to a neutral pose
    rospy.loginfo("Sending robot to neutral pose")
    send_to_neutral()
    rospy.sleep(2.0)

    # Creating a JointCommand message to publish commands
    rospy.loginfo("Commanding...\n")
    pubmsg = JointCommand()

    # names of joints (has to be 7 and in the same order as the command fields (positions, velocities, efforts))
    pubmsg.names = names

    # capture the video from the webcam
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():  # if the camera can't be opened exit the program
        print("Cannot open camera")
        exit()

    while not rospy.is_shutdown():

        # computed  delta time for FPS capping
        delta_time = time.perf_counter() - prev_time

        ret, frame = cap.read()  # read a frame from the webcam

        if not ret:  # if a frame can't be read, exit the program
            print("Can't receive frame from camera/stream end")
            break

        # if frame comes from a webcam flip it to mirror the image
        if source == 0:
            frame = cv2.flip(frame, 1)

        if delta_time >= time_lim:  # if the time passed is bigger or equal than the frame time, process the frame
            prev_time = time.perf_counter()

            # compute the actual frame rate per second (FPS) of the webcam video capture stream, and show it
            ctime = time.perf_counter()
            fps = 1.0 / float(ctime - ptime)
            ptime = ctime

            # find the hands in the image
            frame = HandDet.findHands(frame=frame, draw=True)

            # find the pose in the image
            frame_pose = PoseDet.findPose(frame=frame, draw=False)

            # extract the pose 2D and 3D keypoints from the frame
            lm_list_pose = PoseDet.findPosePosition(
                frame, additional_info=True, draw=False)
            lm_3dlist_pose = PoseDet.find3DPosePosition()

            # extract the hand 2D keypoints from the frame
            hand_lmlist, frame = HandDet.findHandPosition(
                frame=frame, hand_num=0, draw=False)

            # If all the keypoints data is available, compute the pose angles, the hand aperture and send it to the robot
            if len(lm_list_pose) > 0 and len(lm_3dlist_pose) > 0 and len(hand_lmlist) > 0:

                elbow_angle_3d = PoseDet.findAngle(
                    frame, 12, 14, 16, angle3d=True, draw=True)
                elbow_angle_3d_second = PoseDet.findAngle(
                    frame, 11, 13, 15, angle3d=True, draw=True)
                elbow_angle_3d_body = PoseDet.findAngle(
                    frame, 23, 11, 12, angle3d=True, draw=True)

                frame, aperture = HandDet.findHandAperture(
                    frame=frame, verbose=True, show_aperture=True)

                # creating a valid angle list for the robot joint, given the extracted angles and hand aperture
                vals = [starting_pose[0] + coefficient_body*((elbow_angle_3d_body-80)/10), (starting_pose[1] + coefficient*((elbow_angle_3d_second-90)/10)),
                        starting_pose[2],
                        (starting_pose[3] + coefficient *
                         ((elbow_angle_3d - 90)/10)),
                        starting_pose[4],
                        (starting_pose[5] + coefficient*(aperture/10)),
                        starting_pose[6]]

                # set the message equal to the joint values
                pubmsg.position = vals
                # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)
                pubmsg.mode = pubmsg.POSITION_MODE
                # publish the message
                pub.publish(pubmsg)
                rate.sleep()

            if show_fps:
                cv2.putText(frame, "FPS:" + str(round(fps, 0)), (10, 400), cv2.FONT_HERSHEY_PLAIN, 2,
                            (255, 255, 255), 1)

            cv2.imshow("Frame (press 'q' to exit)", frame)

        # if the key "q" is pressed on the keyboard, the program is terminated
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return


if __name__ == '__main__':
    # we "capped" the script at 10 FPS for performance reasons
    # we selected the source = 0 (webcam) because it is the default webcam on all computers
    main(fps_cap=10, show_fps=True, source=0)
