#!/usr/bin/env python
# run with turtlesim
import rospy
import time
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        # self.goal_subscriber = rospy.Subscriber('/turtle1/goal',Pose,self.goalcallback)
        self.pose = Pose()
        self.goal = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        # self.pose.y = 10 - round(self.pose.y, 4) # jpw to unifine the coordinate?

    # def goalcallback(self, data):
    #     # self.goal = data
    #     # self.goal.x = round(self.pose.x, 4)
    #     # self.goal.y = round(self.pose.y, 4)
    #     self.goal = data
    #     self.goal.x = round(self.goal.x, 4)
    #     self.goal.y = round(self.goal.y, 4)
    #     # goal_pose = Pose()
    #     # goal_pose.x = input("Set your x goal:")
    #     # goal_pose.y = input("Set your y goal:")

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        # goal_pose.x = self.goal.x#input("Set your x goal:")
        # goal_pose.y = self.goal.y#input("Set your y goal:")
        goal_pose.x = 3
        goal_pose.y = 2
        print("x:",goal_pose.x)
        print("y:",goal_pose.y)
        print(1)
        print("theta",atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
        distance_tolerance = 0.5#input("Set your tolerance:")
        vel_msg = Twist()

        

        while abs((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta))>0.1:
            # vel_msg.angular.z = 0.2 #turtlebot
            vel_msg.angular.z = 1 #turtlesim
            # vel_msg.angular.z = 0.2 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            self.velocity_publisher.publish(vel_msg)
            print("line_theta:",atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x))
            print("robot_theta:",self.pose.theta)
            print("goal_xy",goal_pose.x,goal_pose.y)
            print("pose_xy",self.pose.x,self.pose.y)
            print((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta))

            self.rate.sleep()
        print("finish rotating")            

        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            # vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            # vel_msg.linear.x = 1.5/10 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            # vel_msg.linear.x = 0.1 #turtlebot3
            vel_msg.linear.x = 1 #turtlsim
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            # vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            vel_msg.angular.z = 4/6 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            # vel_msg.linear.x = 0.05

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        print("finish forward")   
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        # rospy.spin()

    def move2target(self,yolo_class):
        if yolo_class == 0:  #cup
            goal_pose = Pose()
            goal_pose.x = 5
            goal_pose.y = 1
        else:
            goal_pose = Pose()
            goal_pose.x = 5
            goal_pose.y = 9
        print("target x:",goal_pose.x)
        print("target y:",goal_pose.y)
        distance_tolerance = 0.5#input("Set your tolerance:")
        vel_msg = Twist()

        

        while abs((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta))>0.1:
            # vel_msg.angular.z = 0.2 #turtlebot
            vel_msg.angular.z = 1 #turtlesim
            # vel_msg.angular.z = 0.2 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            self.velocity_publisher.publish(vel_msg)
            print("line_theta:",atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x))
            print("robot_theta:",self.pose.theta)
            print("goal_xy",goal_pose.x,goal_pose.y)
            print("pose_xy",self.pose.x,self.pose.y)
            print((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta))

            self.rate.sleep()
        print("finish rotating")            

        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            # vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            # vel_msg.linear.x = 1.5/10 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            # vel_msg.linear.x = 0.1 #turtlebot3
            vel_msg.linear.x = 1 #turtlsim
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            # vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            vel_msg.angular.z = 4/6 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            # vel_msg.linear.x = 0.05

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        print("finish forward")   
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        # rospy.spin()

if __name__ == '__main__':   #only excecute once
    try:
        #Testing our function
        x = turtlebot()
        time.sleep(1) #need some time to initlize the subscriber
        x.move2goal()
        print('Finish to goal!')

        x.move2target(0)
        rospy.spin()
    except rospy.ROSInterruptException: pass