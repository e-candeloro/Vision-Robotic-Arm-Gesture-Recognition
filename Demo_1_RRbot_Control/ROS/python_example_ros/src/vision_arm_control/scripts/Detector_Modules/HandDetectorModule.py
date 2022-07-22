import mediapipe as mp
import matplotlib.pyplot as plt
import numpy as np
import cv2

import time


class HandDetector():
    def __init__(self, mode=False, maxHands=1, modCompl=1, detCon=0.5, trackCon=0.5):
        """Hand detector class that is used to detect the hand keypoints.

        Args:
            mode (bool, optional): If set to false, the solution treats the input images as a video stream. It will try to detect hands in the first input images, and upon a successful detection further localizes the hand landmarks. In subsequent images, once all max_num_hands hands are detected and the corresponding hand landmarks are localized, it simply tracks those landmarks without invoking another detection until it loses track of any of the hands. This reduces latency and is ideal for processing video frames. If set to true, hand detection runs on every input image, ideal for processing a batch of static, possibly unrelated, images. Default to false.
            
            maxHands (int, optional): Maximum number of hands to detect. Default to 1.
            
            modCompl (int, optional): Complexity of the hand landmark model: 0 or 1. Landmark accuracy as well as inference latency generally go up with the model complexity. Default to 1.
            
            detCon (float, optional): Minimum confidence value ([0.0, 1.0]) from the hand detection model for the detection to be considered successful. Default to 0.5.
            
            trackCon (float, optional): Minimum confidence value ([0.0, 1.0]) from the landmark-tracking model for the hand landmarks to be considered tracked successfully, or otherwise hand detection will be invoked automatically on the next input image. Setting it to a higher value can increase robustness of the solution, at the expense of a higher latency. Ignored if static_image_mode is true, where hand detection simply runs on every image. Default to 0.5.
        """
        self.mode = mode  # static image mode,
        self.maxHands = maxHands  # max number of hands to track
        self.modCompl = modCompl  # complexity of the model (can be 0 or 1)
        self.detCon = detCon  # detection confidence threshold
        self.trackCon = trackCon  # tracking confidence threshold

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode,
                                        max_num_hands=self.maxHands,
                                        model_complexity=self.modCompl,
                                        min_detection_confidence=self.detCon,
                                        min_tracking_confidence=self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, frame, draw=True, return_handedness=False):
        """ Detects the hands in the input image.

        Args:
            frame (OpenCV BGR image): Input image.
            draw (bool, optional): If set to true, draw the hand(s) keypoints and connections. Defaults to True.
            return_handedness (bool, optional): Returns the list of score and label for right handedness.ATTENTION: if the input image is not flipped, returns the label Right for the left hand and vice-versa!!!. Defaults to False.

        Returns:
            frame:  opencv image in BGR with keypoints drawn if draw is set to true
            right_handedness (optional): list of scores and labels for hand handedness.
        """
        '''
        Detects the hands and draws keypoints of the hands given and input image.
        :param: img (opencv image in BGR)
        :param: draw (boolean, draw the keypoint if set to true, default is true)
        :returns: img (opencv image in BGR with keypoints drawn if draw is set to true)
        '''
        
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLMs in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLMs,
                                               self.mpHands.HAND_CONNECTIONS)
        
        if return_handedness:
            return frame, self.results.multi_handedness
        else:
            return frame

    def findHandPosition(self, frame, hand_num=0, draw=True):
        '''
        Given and image, returns the hand keypoints position in the format of a list of lists
        [[id_point0, x_point0, y_point0], ..., [id_point19, x_point19, y_point19]]
        The number of hand keypoints are 20 in total.
        Keypoints list and relative position are shown in the example notebook and on this site: https://google.github.io/mediapipe/solutions/hands.html

        :param: img (opencv BGR image)
        :param: hand_num (hand id number to detect, default is zero)
        :draw: bool (draws circles over the hand keypoints, default is true)

        :returns: 
            lm_list (list of lists of keypoints)
            img
        '''
        self.lm_list = []
        h, w, c = frame.shape

        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_num]
            for id_point, lm in enumerate(hand.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lm_list.append([id_point, cx, cy])
                if draw:
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        return self.lm_list, frame

    def findHand3DPosition(self, hand_num=0, draw=False):
        '''
        Find the hand 3d positions on the referred detected hand in real-world 3D coordinates 
        that are in meters with the origin at the hand's approximate geometric center.
        Please refer to the documentation for further details: 
        https://google.github.io/mediapipe/solutions/hands.html#multi_hand_world_landmarks


        :param: hand_num (hand id number to detect, default is zero)
        :draw: bool (draws a 3d graph of the predicted locations in world coordinates of the hand keypoints, default is False)

        :returns: list of lists of 3d hand keypoints in the format [[id_point, x_point,y_point,z_point]]
        '''
        self.lm3d_list = []
        if self.results.multi_hand_world_landmarks:
            hand3d = self.results.multi_hand_world_landmarks[hand_num]
            for id_point, lm in enumerate(hand3d.landmark):
                self.lm3d_list.append([id_point, lm.x, lm.y, lm.z])
            if draw:
                self.mpDraw.plot_landmarks(
                    hand3d, self.mpHands.HAND_CONNECTIONS, azimuth=5)
        return self.lm3d_list

    def findHandAperture(self, frame, verbose=False, show_aperture=True, aperture_range: list = [0.4, 1.7]):
        '''
        Finds the normalized hand aperture as distance between the mean point of the hand tips and the mean wrist and thumb base point divided by the palm lenght.

        Parameters
        ----------
        frame: opencv image array
            contains frame to be processed
        verbose: bool
            If set to True, prints the hand aperture value on the frame (default is False)
        show_aperture: bool
            If set to True, show the hand aperture with a line
        aperture_range: list of 2 floats containing the min aperture and max aperture to remap from 0 to 1

        default: [0.4, 1.7] gets remapped to [0, 1] 

        Returns
        --------
        frame, hand aperture (aperture)
        In case the aperture can't be computed, the value of aperture will be None
        '''
        aperture = None

        thumb_cmc_lm_array = np.array([self.lm_list[1][1:]])[0]
        wrist_lm_array = np.array([self.lm_list[0][1:]])[0]
        lower_palm_midpoint_array = (thumb_cmc_lm_array + wrist_lm_array) / 2

        index_mcp_lm_array = np.array([self.lm_list[5][1:]])[0]
        pinky_mcp_lm_array = np.array([self.lm_list[5][1:]])[0]
        upper_palm_midpoint_array = (
            index_mcp_lm_array + pinky_mcp_lm_array) / 2

        # compute palm size as l2 norm between the upper palm midpoint and lower palm midpoint
        palm_size = np.linalg.norm(
            upper_palm_midpoint_array - lower_palm_midpoint_array, ord=2)
        # print(f"palm size:{palm_size}")

        index_tip_array = np.array([self.lm_list[8][1:]])[0]
        middle_tip_array = np.array([self.lm_list[12][1:]])[0]
        ring_tip_array = np.array([self.lm_list[16][1:]])[0]
        pinky_tip_array = np.array([self.lm_list[20][1:]])[0]

        hand_tips = np.array([index_tip_array,
                              middle_tip_array,
                              ring_tip_array,
                              pinky_tip_array])
        # print(f"hand_tips: {hand_tips}")

        tips_midpoint_array = np.mean(hand_tips, axis=0)
        # print(f"tips_midpoint_array:{tips_midpoint_array}")

        # compute hand aperture as l2norm between hand tips midpoint and lower palm midpoint
        # normalize by palm size computed before
        aperture = np.linalg.norm(
            tips_midpoint_array - lower_palm_midpoint_array, ord=2)/palm_size

        aperture_norm = np.round(
            np.interp(aperture, aperture_range, [0, 100]), 1)

        if verbose:
            cv2.putText(frame, "HAND APERTURE:" + str(aperture_norm), (10, 40),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 1, cv2.LINE_AA)
        if show_aperture:
            frame = cv2.line(frame, tuple(tips_midpoint_array.astype(int)),
                             tuple(lower_palm_midpoint_array.astype(int)), (255, 0, 0), 3)

        return frame, aperture_norm

# ---------------------------------------------------------------
# MAIN SCRIPT EXAMPLE FOR REAL-TIME HAND TRACKING USING A WEBCAM
# ---------------------------------------------------------------

def main(camera_source=0, show_fps=True, verbose=False):

    assert camera_source >= 0, f"source needs to be greater or equal than 0\n"

    ctime = 0  # current time (used to compute FPS)
    ptime = 0  # past time (used to compute FPS)

    cv2.setUseOptimized(True)

    # capture the input from the default system camera (camera number 0)
    cap = cv2.VideoCapture(camera_source)
    detector = HandDetector(detCon=0.7, trackCon=0.7)

    if not cap.isOpened():  # if the camera can't be opened exit the program
        print("Cannot open camera")
        exit()

    while True:  # infinite loop for webcam video capture

        ret, frame = cap.read()  # read a frame from the webcam

        if not ret:  # if a frame can't be read, exit the program
            print("Can't receive frame from camera/stream end")
            break

        frame, handness_list = detector.findHands(frame=frame, return_handedness=True)
        hand_lmlist, frame = detector.findHandPosition(
            frame=frame, hand_num=0, draw=False)

        if len(hand_lmlist) > 0:
            frame, aperture = detector.findHandAperture(
                frame=frame, verbose=True, show_aperture=True)

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
    main(camera_source=0)
    