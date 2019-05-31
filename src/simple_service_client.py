#!/usr/bin/env python

import rospy

from ubiquitous_display_pantilt.srv import AddPoints
from geometry_msgs.msg import Point

import sys


if __name__ == '__main__':

    rospy.init_node('simple_service_client_for_pantilt')

    set_pantilt = rospy.ServiceProxy('action_plan_pantilt', AddPoints)

    set_point = Point()

    set_point.x = float(sys.argv[1])
    set_point.y = float(sys.argv[2])
    set_point.z = float(sys.argv[3])

    response = set_pantilt(set_point)

    if response.success:
        rospy.loginfo('set [%f, %f, %f] sucess' % (set_point.x, set_point.y, set_point.z))
    else:
        rospy.logerr('failed')
