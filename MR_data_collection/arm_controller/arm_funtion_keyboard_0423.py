
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float64
import time

rospy.init_node('arm_controller')

# Define the joint positions for each action
positions = {
    'arm_up_close': [1.0, 0.9, 3.14], #big arm
    'arm_up_open': [1.0, 0.9, -3.14],
    'arm_down_close': [1.0, 0.0,3.14],
    'arm_down_open': [1.0, 0.0, -3.14]
    # 'arm_up_close': [1.0, 0.9, 0.8], #small arm
    # 'arm_up_open': [1.0, 0.9, -2.5],
    # 'arm_down_close': [1.0, 0.0, 0.8],
    # 'arm_down_open': [1.0, 0.0, -2.5]
    # 'arm_up_close': [1.0, 0.9, 1.3],
    # 'arm_up_open': [1.0, 0.9, -3.14],
    # 'arm_down_close': [1.0, 0.0, 1.3],
    # 'arm_down_open': [1.0, 0.0, -3.14]
}

actions = {
    'pick_up': '1',
    'put_down': '2'
}

pub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=10)
pub_joint_move_time = rospy.Publisher('/joint_move_time', Float64, queue_size=10)

rate = rospy.Rate(10)

def pick_up():
   # move arm up and open
    msg = Float64MultiArray()
    msg.data = positions['arm_up_open']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(1))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_up_open']))
    time.sleep(1)

    # move arm down and open
    msg = Float64MultiArray()
    msg.data = positions['arm_down_open']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(1))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_down_open']))
    time.sleep(1)

    # move arm down and close
    msg = Float64MultiArray()
    msg.data = positions['arm_down_close']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(2.0))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_down_close']))
    time.sleep(2)

    # move arm up and close
    msg = Float64MultiArray()
    msg.data = positions['arm_up_close']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(1.0))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_up_close']))
    time.sleep(1)


def put_down():
    # move arm up and close
    msg = Float64MultiArray()
    msg.data = positions['arm_up_close']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(1.0))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_up_close']))
    time.sleep(1)

    # move arm down and close
    msg = Float64MultiArray()
    msg.data = positions['arm_down_close']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(1.0))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_down_close']))
    time.sleep(1)

    # move arm down and open
    msg = Float64MultiArray()
    msg.data = positions['arm_down_open']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(2.0))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_down_open']))
    time.sleep(2)

    # move arm up and open
    msg = Float64MultiArray()
    msg.data = positions['arm_up_open']
    pub.publish(msg)
    pub_joint_move_time.publish(Float64(1.0))
    rospy.loginfo('Published joint positions: {}'.format(positions['arm_up_open']))
    time.sleep(1)

while not rospy.is_shutdown():
    # Wait for keyboard input to select the action
    action = raw_input('Enter action (pick_up=1, put_down=2): ')

    # Check if the selected action exists
    if action in actions.values():
        if action == actions['pick_up']:
            pick_up()
        elif action == actions['put_down']:
            put_down()
    else:
        rospy.logwarn('Unknown action: {}'.format(action))

    rate.sleep()

