#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from location.msg import *
from location.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose


class Current_Register:
    pos = None  # type: Pose
    pub = None  # type: rospy.Publisher

    def start(self):
        def amcl_pose_callback(data):
            # type: (PoseWithCovarianceStamped) -> None
            self.pos = data.pose.pose

        def current_register_callback_(data):
            # type: (String) -> None
            if self.pos is not None and self.pub is not None:
                self.pub.publish(Location(data.data, self.pos))

        rospy.init_node('current_register', anonymous=False)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
        rospy.Subscriber("/location/register/current", String, current_register_callback_)
        self.pub = rospy.Publisher("/location/register", Location, queue_size=10)
        rospy.spin()


if __name__ == "__main__":
    Current_Register().start()
