#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from location.msg import *
from location.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


def start():
    '''
    rospy.wait_for_service("location/request")
    try:
        request = rospy.ServiceProxy('location/request', Request_Location)
        result = request(std_msgs.msg.Header(), "abc")
        return result
    except rospy.ServiceException:
        print("なし")
    '''

    rospy.init_node('location_concepts/current_register', anonymous=False)
    p = rospy.Publisher("location/register", Location, queue_size=10)
    rospy.Rate(1).sleep()
    p.publish(Location("abc", Point2D(1, 2)))


if __name__ == "__main__":
    start()
