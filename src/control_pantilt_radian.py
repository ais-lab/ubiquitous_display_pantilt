#!/usr/bin/env python
import cv2
import numpy as np
from ubiquitous_display_pantilt.msg import Pantilt
from dynamixel_controllers.srv import SetSpeed
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int16
from std_msgs.msg import Int16MultiArray
import rospy
import time
import math
import tf

MAX_PAN_RADIAN = 1.24 #71
MIN_PAN_RADIAN = 3.8  #217

MAX_TILT_RADIAN = 1.04 # 2.96
MIN_TILT_RADIAN = 0.52 #29

def checkPanRadian(radian):

    radian = constrain(radian, -MIN_PAN_RADIAN, MAX_PAN_RADIAN)

    return radian

def checkTiltRadian(radian):

    radian = constrain(radian, -MIN_TILT_RADIAN, MAX_TILT_RADIAN)

    return radian

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def get_tilt_position():

    listener = tf.TransformListener()
    listener.waitForTransform('base_footprint', '/projector_link', rospy.Time(), rospy.Duration(4.0))

    try:
      now = rospy.Time.now()
      listener.waitForTransform('base_footprint', '/projector_link', now, rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/base_footprint', '/projector_link', now)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("TF Exception")
      return

    e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    # print "current sngle: pan" ,math.degrees(e[0]),math.degrees(e[1]),math.degrees(e[2])
    #print trans[2]
    # print e
    return -e[0]

def get_pan_position():

    listener = tf.TransformListener()
    listener.waitForTransform('base_footprint', '/projector_link', rospy.Time(), rospy.Duration(4.0))

    try:
      now = rospy.Time.now()
      listener.waitForTransform('base_footprint', '/projector_link', now, rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/base_footprint', '/projector_link', now)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("TF Exception")
      return

    e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    # print "current sngle tilt" ,math.degrees(e[0]),math.degrees(e[1]),math.degrees(e[2])
    #print trans[2]
    # print e
    return e[2]


def callback(data):
    pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
    #pub_proj = rospy.Publisher('proj_array', Int16MultiArray, queue_size=10)
    pub_int = rospy.Publisher('finish_pantilt', Int16, queue_size=10)

    float_pan = Float64()
    float_tilt = Float64()

    pan_speed = data.speed.x
    tilt_speed = data.speed.y
    set_pan_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)
    set_pan_speed(pan_speed)
    set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
    set_tilt_speed(tilt_speed)


    pan_degree = data.position.x
    tilt_degree = data.position.y
    z_point = data.position.z

    float_pan = pan_degree
    target_pan = float_pan

    float_tilt = tilt_degree
    target_tilt = float_tilt


    pub_pan.publish(float_pan)
    pub_tilt.publish(float_tilt)

    offset_tilt = 0.04 #2.5 degree
    offset_pan = 0.04
    while True:
      #print "pass"
      current_tilt = get_tilt_position()
      current_pan = get_pan_position()
      print "c", current_pan, current_tilt
      print "t", target_pan, target_tilt
      if current_tilt < target_tilt + offset_tilt and current_tilt > target_tilt - offset_tilt and current_pan < target_pan + offset_pan and current_pan > target_pan - offset_pan:
        break

    rospy.set_param('control_pantilt/current_tilt_degree', current_tilt)
    rospy.set_param('control_pantilt/current_pan_degree', current_pan)
    rospy.set_param('control_pantilt/current_x_position', 0.0)
    rospy.set_param('control_pantilt/current_y_position', 0.0)

    fin_message = Int16()
    fin_message = 1
    pub_int.publish(fin_message)
    print ("finish pantilt")





def ud_pantilt():
    pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

    rospy.init_node('ubiquitous_display_pantilt_radian', anonymous=True)

#    listener = tf.TransformListener()
#    listener.waitForTransform('ud_base_footprint', '/ud_pt_projector_link', rospy.Time(), rospy.Duration(4.0))
#    listener.waitForTransform('ud_base_footprint', '/ud_pt_plate_link', rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber('pantilt_radian_msg', Pantilt, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")

if __name__ == '__main__':

    ud_pantilt()
