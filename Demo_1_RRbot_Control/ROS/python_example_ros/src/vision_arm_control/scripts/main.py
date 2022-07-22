#!/usr/bin/env python3

import cv2
import time
import math

from Detector_Modules.HandDetectorModule import HandDetector as hdm
from Detector_Modules.PoseDetectorModule import poseDetector as pdm

from python_robot_pub.ros_publisher import ROSPublisher as RosPub


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

    ctime = 0  # current time (used to compute FPS)
    ptime = 0  # past time (used to compute FPS)
    prev_time = 0  # previous time variable, used to set the FPS limit

    fps_lim = fps_cap  # FPS upper limit value, needed for estimating the time for each frame and increasing performances

    time_lim = 1. / fps_lim  # time window for each frame taken by the webcam

    #initialize hand and pose detector objects
    HandDet = hdm()
    PoseDet = pdm()
    
    #initialize the publishers objects
    J1_Pub = RosPub(node_name="joint1_publisher", topic="/rrbot/joint1_position_controller/command", data="Float64", queue=10, signal=True)
    time.sleep(0.5)
    J2_Pub = RosPub(node_name="joint2_publisher", topic="/rrbot/joint2_position_controller/command", data="Float64", queue=10, signal=True)

    cv2.setUseOptimized(True)  # enable OpenCV optimization

    # capture the input from the default system camera (camera number 0)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():  # if the camera can't be opened exit the program
        print("Cannot open camera")
        exit()

    while True:  # infinite loop for webcam video capture

        # computed  delta time for FPS capping
        delta_time = time.perf_counter() - prev_time

        ret, frame = cap.read()  # read a frame from the webcam

        if not ret:  # if a frame can't be read, exit the program
            print("Can't receive frame from camera/stream end")
            break

        if delta_time >= time_lim:  # if the time passed is bigger or equal than the frame time, process the frame
            prev_time = time.perf_counter()

            # compute the actual frame rate per second (FPS) of the webcam video capture stream, and show it
            ctime = time.perf_counter()
            fps = 1.0 / float(ctime - ptime)
            ptime = ctime

            frame = HandDet.findHands(frame=frame, draw=True)
            frame = PoseDet.findPose(frame=frame, draw=False)
            pose_lmlist = PoseDet.findPosePosition(frame=frame, draw=False)
            hand_lmlist, frame = HandDet.findHandPosition(
            frame=frame, hand_num=0, draw=False)

            if len(hand_lmlist) > 0:
                frame, aperture = HandDet.findHandAperture(
                    frame=frame, verbose=True, show_aperture=True)
                J1_Pub.publish_once(message=3.0 - 0.3*(aperture/10))
                J2_Pub.publish_once(message=3.0 - 0.3*(aperture/10))
                
            if len(pose_lmlist) > 0:
                elbow_angle = PoseDet.findAngle(frame, 12, 14, 16, angle3d=False, draw=True)
                elbow_angle_rad = math.radians(elbow_angle)
                shoulder_angle = PoseDet.findAngle(frame, 24, 12, 14, angle3d=False, draw=True)
                

            if show_fps:
                cv2.putText(frame, "FPS:" + str(round(fps, 0)), (10, 400), cv2.FONT_HERSHEY_PLAIN, 2,
                            (255, 255, 255), 1)

            # show the frame on screen
            cv2.imshow("Frame (press 'q' to exit)", frame)

        # if the key "q" is pressed on the keyboard, the program is terminated
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    #killing the publishers after closing the camera window
    J1_Pub.killnode()
    J2_Pub.killnode()
    print("Killed Publisher Nodes")

    return


if __name__ == '__main__':
    main(fps_cap=30, show_fps=True, source=0)
