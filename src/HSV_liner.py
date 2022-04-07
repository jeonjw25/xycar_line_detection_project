import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

from liner import Liner

class HSVLiner(Liner):    
    def callback(self, msg):
        src = self.imgmsg2numpy(msg)
        src = cv2.GaussianBlur(src, (0, 0), 1.0)
        #gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([255, 160, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        roi = mask[480:500, :]

        left = -1 
        right = -1

        for l in range(300-1, 20-1, -1):
            area = roi[5:15, l - 20: l]
            if cv2.countNonZero(area) > 100:
                left = l
                break
        
        for r in range(520, 800):
            area = roi[5:15, r - 20: r]
            if cv2.countNonZero(area) > 100:
                right = r
                break

        if left != -1:
            lsquare = cv2.rectangle(src, (left - 20, 5 + 480), (left, 15 + 480), (0, 255, 0), 3)
        if right != -1:
            rsquare = cv2.rectangle(src, (right - 20, 5 + 480), (right, 15 + 480), (0, 255, 0), 3)
        

        cv2.imshow('src', src)
        cv2.imshow("mask", mask)
        cv2.waitKey(10)

liner = HSVLiner('HSV_liner')
liner.run()

