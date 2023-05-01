#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray,Float64

rospy.init_node('joint_trajectory_publisher')

# Define the joint positions for each action
actions = {
    # '1': [1.0, 1.0, 1.0,],  #small arm
    # '2': [1.0, 0.0, 1.0],
    #  '3': [1.0, 0.0, -3.14]
    '1': [1.0, 1.0, 3.14],   # big arm
    '2': [1.0, 0.0, 3.14],
     '3': [1.0, 0.0, -3.14]
}

pub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=10)
pub_joint_move_time = rospy.Publisher('/joint_move_time', Float64, queue_size=10)



rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # Wait for keyboard input to select the action
    action = raw_input('Enter action (1, 2, 3): ')

    # Check if the selected action exists
    if action in actions:
        msg = Float64MultiArray()
        msg.data = actions[action]  # set joint positions for the selected action
        pub.publish(msg)
        pub_joint_move_time.publish(Float64(1.0))
        rospy.loginfo('Published action: {}'.format(action))
    else:
        rospy.logwarn('Unknown action: {}'.format(action))

    rate.sleep()