#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('joint_trajectory_publisher')

pub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=10)

msg = Float64MultiArray()
msg.data = [1.0, 1.0, 1.0]  # set joint positions

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()