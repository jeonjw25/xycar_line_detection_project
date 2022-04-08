#!/usr/bin/env python
import time
from abc import abstractmethod
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PID import PID
from ego_controller import egoController

class Liner:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.sub = rospy.Subscriber('/usb_cam/image_raw', 
        Image, self.callback, queue_size=1)
        self.bridge = CvBridge()
        # 0.5, 0.0005, 0.00005  
        # 0.3, 0.0005, 0.0002
        # 24, 0.31, 0.0005, 0.0002
        # 26, 0.30802, 0.0005, 0.00037
        # 26, 0.30803, 0.0005, 0.000372
        # 26, 0.308025, 0.0005, 0.000372
        # 26, 0.308025, 0.00051, 0.000373
        self.s_pid = PID(0.3080265, 0.00051, 0.000373)
        self.c_pid = PID(0.8, 0.0005, 0.00005)
        self.controller = egoController()
        self.controller.steer(0)
        time.sleep(1)
    
    def imgmsg2numpy(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    @abstractmethod
    def callback(self, msg):
        pass

    @staticmethod
    def run():
        rospy.spin()
