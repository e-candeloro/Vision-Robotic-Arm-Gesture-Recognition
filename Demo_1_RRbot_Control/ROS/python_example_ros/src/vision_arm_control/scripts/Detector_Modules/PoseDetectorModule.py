import mediapipe as mp
import numpy as np
import cv2

import time


class poseDetector():
    def __init__(self, mode=False, modCompl=1, upBody=False, smooth=True, segm=False, smooth_seg=True, detCon=0.5, trackCon=0.5):
        """Pose detector class that is used to detect the position of the body keypoints.

        Args:
            mode (bool, optional): If set to True, enables static image mode. Defaults to False.
            
            modCompl (int, optional): Complexity of the pose landmark model: 0, 1 or 2. Landmark accuracy as well as inference latency generally go up with the model complexity. Default to 1.
            
            upBody (bool, optional): If set to True, set detection for frames containing only upper body . Defaults to False.
            
            smooth (bool, optional):If set to true, the solution filters pose landmarks across different input images to reduce jitter, but ignored if mode is also set to true. Default to true.
            
            segm (bool, optional):If set to true, in addition to the pose landmarks the solution also generates the segmentation mask. Default to false.
            
            smooth_seg (bool, optional): If set to true, the solution filters segmentation masks across different input images to reduce jitter. Ignored if enable_segmentation is false or static_image_mode is true. Default to true.
            
            detCon (float, optional): Minimum confidence value ([0.0, 1.0]) from the person-detection model for the detection to be considered successful. Default to 0.5
            
            trackCon (float, optional): Minimum confidence value ([0.0, 1.0]) from the landmark-tracking model for the pose landmarks to be considered tracked successfully, or otherwise person detection will be invoked automatically on the next input image. Setting it to a higher value can increase robustness of the solution, at the expense of a higher latency. Ignored if static_image_mode is true, where person detection simply runs on every image. Default to 0.5.
        """
        self.mode = mode  # static image mode
        self.modCompl = modCompl
        self.upBody = upBody
        self.smooth = smooth
        self.segm = segm
        self.smooth_seg = smooth_seg
        self.detCon = detCon  # detection confidence threshold
        self.trackCon = trackCon  # tracking confidence threshold

        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(static_image_mode=self.mode,
                                     model_complexity=self.modCompl,
                                     smooth_landmarks=self.smooth,
                                     enable_segmentation=self.segm,
                                     smooth_segmentation=self.smooth_seg,
                                     min_detection_confidence=self.detCon,
                                     min_tracking_confidence=self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findPose(self, frame, draw=True):
        """
        Detects the pose of the person in the given image.

        Args:
            frame (OpenCV BGR frame): img (opencv image in BGR)
            draw (bool, optional): draw the keypoint if set to true. Defaults to True.

        Returns:
            opencv image in BGR with keypoints drawn if draw is set to true
        """
        
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)

        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
        return frame

    def findPosePosition(self, frame, additional_info=False, draw=True):
        '''
        Given and image, returns the pose keypoints position in the format of a list of lists
        [[id_point0, x_point0, y_point0], ...]
        If additional info is True, returns a list of list in the format
        [[id_point0, x_point0, y_point0, zpoint0, visibility], ...]

        Keypoints list  are shown on this site: https://google.github.io/mediapipe/images/mobile/pose_tracking_full_body_landmarks.png

        :param: additional_info (returns z and visibility in the keypoint list. Default is False)
        :param: frame(opencv BGR image)
        :draw: bool (draws circles over the keypoints. Default is True)

        :returns: 
            lm_list (list of lists of keypoints)
            img
        '''
        self.lm_list = []
        h, w, c = frame.shape

        if self.results.pose_landmarks:
            pose = self.results.pose_landmarks
            for id_point, lm in enumerate(pose.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                if additional_info:
                    cz = lm.z
                    vis = lm.visibility
                    self.lm_list.append([id_point, cx, cy, cz, vis])
                else:
                    self.lm_list.append([id_point, cx, cy])

                if draw:
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        return self.lm_list

    def find3DPosePosition(self, additional_info=False, draw=False):
        '''
        Given and image, returns the 3D pose keypoints position in the format of a list of lists
        [[id_point0, x_point0, y_point0, zpoint0], ...]
        The keypoints are in world coordinates and in meter, with the origin in the middle point of the hips
        If additional info is True, returns a list of list in the format
        [[id_point0, x_point0, y_point0, zpoint0, visibility], ...]

        Keypoints list  are shown on this site: https://google.github.io/mediapipe/images/mobile/pose_tracking_full_body_landmarks.png

        :param: additional_info (returns visibility in the keypoint list. Default is False)
        :draw: bool (draws a matplotlib 3d graph of all the keypoints in world coordinates. Default is False)

        :returns: 
            lm_3dlist (list of lists of keypoints)
        '''
        self.lm_3dlist = []

        if self.results.pose_landmarks:
            pose = self.results.pose_world_landmarks
            for id_point, lm in enumerate(pose.landmark):
                cx, cy, cz = lm.x, lm.y, lm.z

                if additional_info:
                    vis = lm.visibility
                    self.lm_3dlist.append([id_point, cx, cy, cz, vis])
                else:
                    self.lm_3dlist.append([id_point, cx, cy, cz])

            if draw:
                self.mpDraw.plot_landmarks(
                    self.results.pose_world_landmarks, self.mpPose.POSE_CONNECTIONS)

        return self.lm_3dlist

    def findAngle(self, frame, p1: int, p2: int, p3: int, angle3d=False, draw=True):
        '''Find the angle between 3 points p1, p2, p3 in succession, where p2 is the point where the angle is measured.
        For the points, only the index number is required. Please refer to this image to select the appriopriate keypoints: https://google.github.io/mediapipe/images/mobile/pose_tracking_full_body_landmarks.png

        Example: elbow angle, given the shoulder keypoint, the elbow keypoint and the wrist keypoint

        :param: frame (opencv frame)
        :p1:first point index
        :p2:second point index
        :p3:third point index
        :angle3d: Bool: performs 3d angle computation, default is False
        :flip_2dangle: Bool: flips the angle computation if it is in 2d, default is False
        :draw: Bool (optional): draws additional info, default is True

        Returns:
            -angle: angle in degrees between the segment s12 and the segment s23 having p2 as vertex, where the angle is located
        '''
        # checks if keypoints values are correct
        assert p1 >= 0 and p1 <= 32, f"p1 must be >=0 and <=32"
        assert p2 >= 0 and p2 <= 32, f"p2 must be >=0 and <=32"
        assert p3 >= 0 and p3 <= 32, f"p3 must be >=0 and <=32"
        

        if angle3d:
            assert len(
            self.lm_3dlist) > 0, f"3D Landmark list is empty, use this function only after using the FindPose and Find3DPosePosition methods"

            x1, y1, z1 = self.lm_3dlist[p1][1:4]
            x2, y2, z2 = self.lm_3dlist[p2][1:4]
            x3, y3, z3 = self.lm_3dlist[p3][1:4]

            v21 = np.array([x1 - x2, y1 - y2, z1 - z2]) * 100
            v32 = np.array([x3 - x2, y3 - y2, z3 - z2]) * 100

        else:
            assert len(
            self.lm_list) > 0, f"Landmark list is empty, use this function only after using the FindPose and FindPosePosition methods"

            x1, y1 = self.lm_list[p1][1:3]
            x2, y2 = self.lm_list[p2][1:3]
            x3, y3 = self.lm_list[p3][1:3]

            v21 = np.array([x1 - x2, y1 - y2]) * 100
            v32 = np.array([x3 - x2, y3 - y2]) * 100

        angle = np.degrees(
            np.arccos(
                np.dot(v21, v32)/(np.linalg.norm(v21, 2)
                                  * np.linalg.norm(v32, 2))
            )
        )

        if draw:
            # draw the angle and the keypoints, and the connections between them.
            cx1, cy1 = self.lm_list[p1][1:3]
            cx2, cy2 = self.lm_list[p2][1:3]
            cx3, cy3 = self.lm_list[p3][1:3]
            
            
            cv2.circle(frame, (cx1, cy1), 5, (255, 0, 255), -1)
            cv2.circle(frame, (cx2, cy2), 5, (255, 0, 255), -1)
            cv2.circle(frame, (cx2, cy2), 10, (255, 0, 255), 1)
            cv2.circle(frame, (cx3, cy3), 5, (255, 0, 255), -1)
            
            cv2.line(frame, (cx2, cy2), (cx3, cy3), (255, 255, 255), 2)
            cv2.line(frame, (cx2, cy2), (cx1, cy1), (255, 255, 255), 2)
            
            cv2.putText(frame, str(round(angle, 0)), (cx2 - 50, cy2 + 50),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2, cv2.LINE_AA)

        return angle

# ---------------------------------------------------------------
# MAIN SCRIPT EXAMPLE FOR REAL-TIME POSE TRACKING USING A WEBCAM
# ---------------------------------------------------------------


def main(camera_source=0, show_fps=True, verbose=False):

    assert camera_source >= 0, f"source needs to be greater or equal than 0\n"

    ctime = 0  # current time (used to compute FPS)
    ptime = 0  # past time (used to compute FPS)

    cv2.setUseOptimized(True)

    # capture the input from the default system camera (camera number 0)
    cap = cv2.VideoCapture(camera_source)
    detector = poseDetector(detCon=0.7, trackCon=0.7, modCompl=1)

    if not cap.isOpened():  # if the camera can't be opened exit the program
        print("Cannot open camera")
        exit()

    while True:  # infinite loop for webcam video capture

        ret, frame = cap.read()  # read a frame from the webcam

        if not ret:  # if a frame can't be read, exit the program
            print("Can't receive frame from camera/stream end")
            break

        frame = detector.findPose(frame=frame,draw=False)
        lm_list = detector.findPosePosition(
            frame, additional_info=True, draw=False)
        lm_3dlist = detector.find3DPosePosition()

        if len(lm_list) > 0 and len(lm_3dlist) > 0:
            elbow_angle_3d = detector.findAngle(
                frame, 12, 14, 16, angle3d=True, draw=True)


        # compute the actual frame rate per second (FPS) of the webcam video capture stream, and show it
        ctime = time.perf_counter()
        fps = 1.0 / float(ctime - ptime)
        ptime = ctime

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

    return


if __name__ == '__main__':
    # change this to zero if you don't have a usb webcam but an in-built camera
    main(camera_source=1)
