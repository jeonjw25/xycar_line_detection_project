#!/usr/bin/env python

import rospy
from xycar_msgs.msg import xycar_motor


class egoController:
    def __init__(self):
        # rospy.init_node('ego_controller')
        self.pub = rospy.Publisher('xycar_motor', xycar_motor)
        self.d = 20

    # def go(self):
    #     msg = xycar_motor()
    #     msg.speed = 1
    #     msg.angle = 0.7
    #     #msg.acceleration = 1
    #     self.pub.publish(msg)

    def steer(self, angle, back=False):
        msg = xycar_motor()
        ab_angle = abs(angle)
        # 30, 25, 20
        if abs(angle) >= 50:
            self.d = 20
        else:
            if self.d > 0:
                self.d -= 1 
        if ab_angle < 40:
            msg.speed = 30 - self.d
        # elif ab_angle < 40:
        #     msg.speed = 30 - self.d
        else:     
            msg.speed = 20
        
        msg.angle = angle
        if back:
            msg.speed = -8
        self.pub.publish(msg)
