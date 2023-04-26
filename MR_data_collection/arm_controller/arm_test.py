#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import sys
import os.path as osp
import numpy as np 
import math
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import JointState, Imu
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
import time
import multiprocessing as mul




import numpy as np
# import cv2 as cv


feature_params = dict(maxCorners=100, qualityLevel=0.01, minDistance=10, blockSize=3)
# KLT光流参数
lk_params = dict(winSize=(31, 31), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
# 随机颜色
color = np.random.randint(0,255,(100,3))

# class OpiticalFlow():
#     max_vel = 0.22
#     max_w = 2.84

#     def __init__(self):
#         self.name = 'burger'
#         rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
#         print 'subcribed image'

#         self.old_frame = None
#         self.frame_with_flow = None
#         self.frame = None
#         self.p0 = None
#         self.p1 = None
#         self.old_gray = None

#         self.frame_gray = None

#     def update_frame(self):
#         if self.frame is not None:
#             if self.old_frame is not None:
#                 self.frame_with_flow = self.frame.copy()
#                 self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
#                 # 计算光流
#                 self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, self.frame_gray, self.p0, None, **lk_params)
#                 # 根据状态选择
#                 good_new = self.p1[st == 1]
#                 good_old = self.p0[st == 1]
#                 v_vector = good_new-good_old
#                 if len(v_vector) < 50 or v_vector is None:
#                     self.old_frame = self.frame.copy()
#                     self.old_gray = cv2.cvtColor(self.old_frame, cv2.COLOR_BGR2GRAY)
#                     self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **feature_params)
#                 else:
#                     # 绘制跟踪线
#                     for i, (new, old) in enumerate(zip(good_new,good_old)):
#                         a,b = new.ravel()
#                         c,d = old.ravel()
#                         self.frame_with_flow = cv2.line(self.frame_with_flow, (a,b),(c,d), color[i].tolist(), 2)
#                         self.frame_with_flow = cv2.circle(self.frame_with_flow,(a,b),5,color[i].tolist(),-1)
#                         # 更新
#                     self.old_gray = self.frame_gray.copy()
#                     self.p0 = good_new.reshape(-1, 1, 2)
#                 return good_new, v_vector

#             else:   # 第一帧
#                 self.old_frame = self.frame.copy()
#                 self.old_gray = cv2.cvtColor(self.old_frame, cv2.COLOR_BGR2GRAY)
#                 self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **feature_params)
#                 print('get first frame')
#                 return None, None
#         else:
#             return None, None

#     def callback(self, imgmsg):
#         # 获取相机图片的回调函数
#         bridge = CvBridge()
#         img = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
#         # print(img)
#         self.frame = img


class RobotInfo():
    def __init__(self):
        self.linear_vel = 0
        self.angular_vel = 0
        self.arm_position = 0
        self.arm_vel = 0
        self.arm_z_position = 0
        self.arm_z_vel = 0
        self.arm_r_position = 0
        self.arm_r_vel = 0

        self.direction = 0
        self.acceleration_x = 0
        
        try:
            rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
            # rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
            rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
            rospy.Subscriber("/imu", Imu, self.imu_callback)
        except BaseException:
            print('Errors uccor when subscribing ')
        else:
            print ('Succefully subscribed robot information '  )

    def model_callback(self, model_states):
        idx = list(model_states.name).index('robot')        
        velocity = model_states.twist[idx]
        self.linear_vel = self.direction * math.sqrt(velocity.linear.x**2+velocity.linear.y**2)
        self.angular_vel = velocity.angular.z

    def joint_state_callback(self, j_state):
        self.arm_vel = j_state.velocity[0]
        self.arm_position = j_state.position[0]
        self.arm_z_vel = j_state.velocity[1]
        self.arm_z_position = j_state.position[1]
        self.arm_r_vel = j_state.velocity[2]
        self.arm_r_position = j_state.position[2]
        
    
    def cmd_vel_callback(self, target_vel):
        self.direction = np.sign(target_vel.linear.x)

    def imu_callback(self, imu_info):
        self.acceleration_x = imu_info.linear_acceleration.x


class RobotController():
    def __init__(self):
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/MYROBOT/arm_controller/command', Float64, queue_size=10)
        self.arm_z_pub = rospy.Publisher('/MYROBOT/arm_controller_z/command', Float64, queue_size=10)
        self.arm_r_pub = rospy.Publisher('/MYROBOT/arm_controller_r/command', Float64, queue_size=10)

        self.vel = Twist()
        self.arm_vel = Float64()        
        self.arm_z_vel = Float64()
        self.arm_r_vel = Float64()
        self.vel.linear.x, self.vel.linear.y, self.vel.linear.z = 0, 0, 0
        self.vel.angular.x, self.vel.angular.y, self.vel.angular.z = 0, 0, 0

        self.car_pos = ModelState()
        self.car_pos.model_name = 'robot'
        self.car_pos.pose.position.x, self.car_pos.pose.position.y, self.car_pos.pose.position.z = 0, 0, 0
        self.car_pos.pose.orientation.x, self.car_pos.pose.orientation.y, self.car_pos.pose.orientation.z, self.car_pos.pose.orientation.w = 0, 0, 0, 0

        # rospy.wait_for_service('/gazebo/set_model_state')
        # self.set_car_pos = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def set_car_vel(self, linear_vel, angular_vel):
        self.vel.linear.x = linear_vel
        self.vel.angular.z = angular_vel
        self.vel_pub.publish(self.vel)
        return True

    def set_car_linear_vel(self, linear_vel):
        self.vel.linear.x = linear_vel
        self.vel_pub.publish(self.vel)
        return True

    def set_car_angular_vel(self, angular_vel):
        self.vel.angular.x = angular_vel
        self.vel_pub.publish(self.vel)
        return True
    
    def set_arm_vel(self, arm_vel):
        self.arm_vel.data = arm_vel
        # print self.arm_vel
        self.arm_pub.publish(self.arm_vel)
        return True    

    def set_arm_z_vel(self, arm_z_vel):
        self.arm_z_vel.data = arm_z_vel
        # print self.arm_vel
        self.arm_z_pub.publish(self.arm_z_vel)
        return True    

    def set_arm_r_vel(self, arm_r_vel):
        self.arm_r_vel.data = arm_r_vel
        # print self.arm_vel
        self.arm_r_pub.publish(self.arm_r_vel)
        return True

    def reset_car_pos(self):
        resp = self.set_car_pos(self.car_pos)
        return resp


def spinner():
    rospy.spin()


if __name__ == '__main__':
    print ('start moving')
    queue = mul.Queue(maxsize=1)

    rospy.init_node('movement_sim', disable_signals=True)
    robot_controller = RobotController()
    robot_info = RobotInfo()
    # optical_flow = OpiticalFlow()
    robot_controller.reset_car_pos()
    
    # # start a new thread for spin
    # spin_thread = mul.Process(target=spinner)
    # spin_thread.start()

    robot_controller.set_arm_vel(0)
    robot_controller.set_arm_z_vel(0)
    robot_controller.set_arm_r_vel(0)
    robot_controller.set_car_vel(0, 0)
    time.sleep(1)

    print('Mobile base linear velocity test: 0.05')
    time.sleep(3)
    print('     Forward')    
    robot_controller.set_car_vel(0.05, 0)
    time.sleep(10)
    robot_controller.set_car_vel(0, 0)
    time.sleep(1)
    print('     Backward')    
    robot_controller.set_car_vel(-0.05, 0)
    time.sleep(10)
    robot_controller.set_car_vel(0, 0)
    time.sleep(1)

    print('Mobile base angular velocity test: 0.3')
    time.sleep(3)
    print('     Counterclockwise')    
    robot_controller.set_car_vel(0, 0.3)
    time.sleep(10)
    robot_controller.set_car_vel(0, 0)
    time.sleep(1)
    print('     Clockwise')    
    robot_controller.set_car_vel(0, -0.3)
    time.sleep(10)
    robot_controller.set_car_vel(0, 0)
    time.sleep(1)

    print('Angular arm rotary joint velocity test: 0.5')
    time.sleep(3)
    print('     Counterclockwise')    
    robot_controller.set_arm_vel(-0.5)
    time.sleep(15)
    robot_controller.set_arm_vel(0)
    time.sleep(1)
    print('     Clockwise loop')    
    robot_controller.set_arm_vel(0.5)
    time.sleep(15)
    robot_controller.set_arm_vel(0)
    time.sleep(1)

    print('Z-axis arm prismatic joint velocity test: 0.02')
    time.sleep(3)
    print('     Upward')    
    robot_controller.set_arm_z_vel(-0.02)
    # if robot_info.arm_z_position>abs(0.151):
    #     robot_controller.set_arm_z_vel(0)
    time.sleep(10)
    robot_controller.set_arm_z_vel(0)    
    time.sleep(1)
    print('     Downward')    
    robot_controller.set_arm_z_vel(0.02)
    # if robot_info.arm_z_position<abs(0.01):
    #     robot_controller.set_arm_z_vel(0)
    time.sleep(10)
    robot_controller.set_arm_z_vel(0)
    time.sleep(1)

    print('Radial arm prismatic joint velocity test: 0.005')
    time.sleep(3)
    print('     Outward')    
    robot_controller.set_arm_r_vel(0.005)
    # if robot_info.arm_r_position>abs(0.039):
    #     robot_controller.set_arm_r_vel(0)
    time.sleep(10)
    robot_controller.set_arm_r_vel(0)
    time.sleep(1)
    print('     Inward')    
    robot_controller.set_arm_r_vel(-0.005)
    # if robot_info.arm_r_position<abs(0.001):
    #     robot_controller.set_arm_r_vel(0)
    time.sleep(10)
    robot_controller.set_arm_r_vel(0)
    time.sleep(1)

    time.sleep(3)
    print('Simulation completed')    