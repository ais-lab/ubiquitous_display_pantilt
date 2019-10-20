#!/usr/bin/env python

import rospy
import numpy as np
import math

from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import JointState
from ubiquitous_display_msgs.srv import PantiltCommand
from ubiquitous_display_msgs.srv import PantiltCommandResponse
from dynamixel_controllers.srv import SetSpeed

class Publishsers():

    def pan_make(self, speed, rad):
        self.set_pan_speed(speed)
        self.pan_pub.publish(rad)

    def tilt_make(self, speed, rad):
        self.set_tilt_speed(speed)
        self.tilt_pub.publish(rad)

class Server(Publishsers):
    def __init__(self):
        self.pan_rad = 0.0
        self.tilt_rad = 0.0

        self.pan_offset = 0.04
        self.tilt_offset = 0.04

        self.rate = rospy.Rate(100)

        self.pan_pub = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
        self.set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
        self.set_pan_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)
        # Declaration Subscriber
        self.ptm_sub = rospy.Subscriber('/pantilt_joint_states', JointState , self.djs_callback)

        # Declaration Service Server
        self.server = rospy.Service("/pantilt_radian_server", PantiltCommand, self.service_callback)

    def djs_callback(self, msg):
        self.pan_rad = msg.position[0]
        self.tilt_rad = msg.position[1]


    def service_callback(self, req):
        return_string = Bool()

        pan_speed = req.pan_speed.data
        tilt_speed = req.tilt_speed.data
        pan_degree = req.pan_degree.data
        tilt_degree = req.tilt_degree.data

        self.pan_make(pan_speed, pan_degree)
        self.tilt_make(tilt_speed, tilt_degree)

        while True:
            if self.pan_rad > pan_degree - self.pan_offset and self.pan_rad < pan_degree + self.pan_offset and self.tilt_rad > tilt_degree - self.tilt_offset and self.tilt_rad < tilt_degree + self.tilt_offset:
                break
            self.rate.sleep()

        return_string.data = True
        return PantiltCommandResponse(return_string)


if __name__ == '__main__':
    rospy.init_node('pantilt_radian_server')

    server = Server()

    rospy.spin()
