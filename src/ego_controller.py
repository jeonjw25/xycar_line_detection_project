#!/usr/bin/env python

import rospy
from xycar_msgs.msg import xycar_motor


class egoController:
    def __init__(self):
        # rospy.init_node('ego_controller')
        self.pub = rospy.Publisher('xycar_motor', xycar_motor)

    # def go(self):
    #     msg = xycar_motor()
    #     msg.speed = 1
    #     msg.angle = 0.7
    #     #msg.acceleration = 1
    #     self.pub.publish(msg)

    def steer(self, angle, back=False):
        msg = xycar_motor()
        ab_angle = abs(angle)
        if ab_angle < 30:
            msg.speed = 30
        elif ab_angle < 40:
            msg.speed = 25
        else:     
            msg.speed = 20
        
        msg.angle = angle
        if back:
            msg.speed = -5
        self.pub.publish(msg)
