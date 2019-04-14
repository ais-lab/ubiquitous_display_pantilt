#!/usr/bin/env python

import rospy
import math
import time
import tf
from ubiquitous_display_pantilt.srv import AddPoints
from ubiquitous_display_pantilt.srv import AddPointsResponse
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64

MAX_PAN_RADIAN = 1.24 #71
MIN_PAN_RADIAN = 3.8  #217

MAX_TILT_RADIAN = 1.04 # 2.96
MIN_TILT_RADIAN = 0.52 #29

### check limit radian for pan and tilt ###
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

### check tf (joint_states) for pan and tilt ###
def get_tilt_position():

    listener = tf.TransformListener()
    listener.waitForTransform('base_footprint', '/tilt_link', rospy.Time(), rospy.Duration(4.0))

    try:
      now = rospy.Time.now()
      listener.waitForTransform('base_footprint', '/tilt_link', now, rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/base_footprint', '/tilt_link', now)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("TF Exception")
      return

    e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    #print "current sngle" ,math.degrees(e[0]),math.degrees(e[1]),math.degrees(e[2])
    #print trans[2]
    return e[1] + math.pi / 2.0

def get_pan_position():

    listener = tf.TransformListener()
    listener.waitForTransform('base_footprint', '/pan_link', rospy.Time(), rospy.Duration(4.0))

    try:
      now = rospy.Time.now()
      listener.waitForTransform('base_footprint', '/pan_link', now, rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/base_footprint', '/pan_link', now)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("TF Exception")
      return

    e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    #print "current sngle" ,math.degrees(e[0]),math.degrees(e[1]),math.degrees(e[2])
    #print trans[2]
    return e[2]

def callback(point):

    pub_pan = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
    pub_tilt = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)

    x_point = point.position.x
    y_point = point.position.y
    z_point = point.position.z

    rad_pan = math.atan2(y_point, x_point)
    #print math.degrees(rad_pan)

    distance = math.sqrt(x_point*x_point + y_point*y_point)
    #print distance

    ud_z_point = 1.21

    rad_tilt = math.atan2(ud_z_point, distance)
    #print math.degrees(rad_tilt)

    float_pan = Float64()
    float_tilt = Float64()

    float_pan = rad_pan - 1.5708
    float_pan = checkPanRadian(float_pan)
    target_pan = float_pan

    float_tilt = rad_tilt
    float_tilt = checkTiltRadian(float_tilt)
    target_tilt = float_tilt


    pub_pan.publish(float_pan)
    pub_tilt.publish(float_tilt)

    offset_tilt = 0.04 #2.5 degree
    offset_pan = 0.04
    while True:
      #print "pass"
      current_tilt = get_tilt_position()
      current_pan = get_pan_position()
      #print "c", current_pan, current_tilt
      #print "t", target_pan, target_tilt
      if current_tilt < target_tilt + offset_tilt and current_tilt > target_tilt - offset_tilt and current_pan < target_pan + offset_pan and current_pan > target_pan - offset_pan:
        # point.success = True
        break

    return AddPointsResponse(True)

def service_server():

    rospy.init_node('simple_service_server_for_pantilt', anonymous=True)

    rospy.Service('action_plan_pantilt', AddPoints, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    service_server()
