#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print (msg.pose.pose)

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, callback)

rospy.spin()
