#!/usr/bin/env python3

"""
RROBOT TOPICS

/rrbot/camera1/camera_info
/rrbot/camera1/image_raw
/rrbot/camera1/image_raw/compressed
/rrbot/camera1/image_raw/compressed/parameter_descriptions
/rrbot/camera1/image_raw/compressed/parameter_updates
/rrbot/camera1/image_raw/compressedDepth
/rrbot/camera1/image_raw/compressedDepth/parameter_descriptions
/rrbot/camera1/image_raw/compressedDepth/parameter_updates
/rrbot/camera1/image_raw/theora
/rrbot/camera1/image_raw/theora/parameter_descriptions
/rrbot/camera1/image_raw/theora/parameter_updates
/rrbot/camera1/parameter_descriptions
/rrbot/camera1/parameter_updates
/rrbot/joint1_position_controller/command
/rrbot/joint1_position_controller/pid/parameter_descriptions
/rrbot/joint1_position_controller/pid/parameter_updates
/rrbot/joint1_position_controller/state
/rrbot/joint2_position_controller/command
/rrbot/joint2_position_controller/pid/parameter_descriptions
/rrbot/joint2_position_controller/pid/parameter_updates
/rrbot/joint2_position_controller/state
/rrbot/joint_states
/rrbot/laser/scan

"""
from ros_publisher import ROSPublisher as rospub
import time

if __name__ == '__main__':
    
    pub = rospub(node_name="rrbot_joint1_publisher", topic="/rrbot/joint2_position_controller/command", data="Float64", queue=10, signal=True)
    
    count = 0
    # pub.publish_recur(message="Hello!", rate_pub=1)
    while count < 10:
        #for i in circle :
        pub.publish_once(message=(0.0+0.3*count))
        time.sleep(1)
        count += 1
        
    pub.killnode()
    print("Terminated")