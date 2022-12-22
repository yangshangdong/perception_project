#!/usr/bin/env python
# move forward and backward specif distance (but I think it remains some problem)
import rospy
from geometry_msgs.msg import Twist
import time

def move_forward():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    #Receiveing the user's input
    print("move robot forward")
    # speed = input("Input your speed:")
    # distance = input("Type your distance:") 
    speed = 0.1
    distance = 1

    vel_msg.linear.x = abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown(): # is a common construct used in ROS Python scripts to run a loop until the ROS node is shut down.

        #Setting the current time for distance calculus
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=float(rospy.Time.now().to_sec())
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        break

def move_backward():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    #Receiveing the user's input
    print("move robot backward")
    # speed = input("Input your speed:")
    # distance = input("Type your distance:")
    speed = 1
    distance = 1

    vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=float(rospy.Time.now().to_sec())
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        break


if __name__ == '__main__':
    try:
        #Testing our function
        move_forward()
        time.sleep(1)

        move_backward()
    except rospy.ROSInterruptException: pass

