#!/usr/bin/env python3

import os
#import cv2
import yaml
import time
import rospy
import numpy as np

from duckietown_msgs.msg import Twist2DStamped, EpisodeStart
#from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from duckietown.dtros import DTROS, NodeType, TopicType
#from duckietown.utils.image.ros import compressed_imgmsg_to_rgb, rgb_to_compressed_imgmsg

class MoveMotor(DTROS):
    """
    Maintained by Zejian@drz
    Move the Duckiebot's trajectory. This also serves as a ROS coding tempelate for the Duckiebot.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:
    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
    Subscribers:
    """
    
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(MoveMotor, self).__init__(node_name=node_name,
                                               node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        #self.veh = rospy.get_namespace().strip("/")
        self.veh = "drz"        
        
        print("DRZ: building process is OK")
        """

	implement your code

	"""


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = MoveMotor(node_name="motor_mover")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    	"""

	implement your code

	"""
    	print("DRZ: building process is OK")
    	rate.sleep()
    # Keep it spinning
    rospy.spin()
    
