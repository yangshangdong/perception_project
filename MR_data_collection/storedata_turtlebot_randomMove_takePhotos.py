


import os
import rospy
import time
from turtlesim.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist, Quaternion,Point
import tf
from math import radians, copysign,sqrt,pow,pi
import random
import PyKDL
     

class RandomMoveWithCamera:
    def __init__(self):
        self.image_received = False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.posecallback)
        self.box1_subscriber = rospy.Subscriber('/turtle1/box1',Pose,self.box1callback)
        self.robot_pose = Pose()
        self.box1_pose = Pose()
        self.move = -1 # intilize number


        # Give the node a name
        rospy.init_node('random_move_node', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        self.rate = 20

        # Create a folder to save the images and the action and position data
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.path = '/home/xtark/Desktop/perception_project/MR_data_collection/realworld_datas_'+ timestamp 
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        
        #txt file to save the data
        self.txt_path = self.path +"/data_collection.txt"  
        with open(self.txt_path, "a") as file:
            file.write("n a robot.x robot.y robot.theta box1.x box1.y \n") 


        for i in range(20):  #random move for i times
            rospy.sleep(0.1)
            #take photo
            self.take_photo(i)
            rospy.loginfo("Take photo "+str(i+1))

            #save the data
            rospy.loginfo("datas: %d %d %.2f %.2f %.4f %.2f %.2f",i+1, self.move, self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta,self.box1_pose.x, self.box1_pose.y)
            with open(self.txt_path, "a") as file:
                file.write("%d %d %.2f %.2f %.4f %.2f %.2f\n" % (i+1, self.move,self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta, self.box1_pose.x, self.box1_pose.y))

            #random move             
            self.random_move()


        #take photo
        self.take_photo(i+1)
        rospy.loginfo("Take photo "+str(i+2))
        #save the data
        rospy.loginfo("datas: %d %d %.2f %.2f %.4f %.2f %.2f",i+2, self.move,self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta,self.box1_pose.x, self.box1_pose.y)
        with open(self.txt_path, "a") as file:
            file.write("%d %d %.2f %.2f %.4f %.2f %.2f\n" % (i+2, self.move,self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta, self.box1_pose.x, self.box1_pose.y))        



        #Callback function implementing the pose value received
    def posecallback(self, data):
        self.robot_pose = data
        self.robot_pose.x = round(self.robot_pose.x, 1)
        self.robot_pose.y = round(self.robot_pose.y, 1)
        self.robot_pose.theta =  round(self.robot_pose.theta, 4)
        # self.pose.y = 10 - round(self.pose.y, 4) # jpw to unifine the coordinate?

    def box1callback(self, data):
        self.box1_pose = data
        self.box1_pose.x = round(self.box1_pose.x, 1)
        self.box1_pose.y = round(self.box1_pose.y, 1)

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.image_received = True
        # rospy.loginfo("Image received!")

    def take_photo(self,i):
        while not self.image_received:
            rospy.sleep(0.1)
        self.image_received = False
        # Save the image with a specific name
        cv2.imwrite(self.path+'/image'+str(i+1)+'.png', self.cv_image)           

    def random_move(self):
        # Use the random.randint() function to choose a number between 0 and 3
        move = random.randint(0, 3)
        if move == 0:
            self.move_forward()
        elif move == 1:
            self.move_backward()
        elif move == 2:
            self.move_left()
        elif move == 3:
            self.move_right()
        self.move = move
        return self.move

    def move_forward(self):
        r = rospy.Rate(self.rate)
        # Set the distance to travel
        self.test_distance =  0.15 # meters
        self.speed = 0.10 # meters per second
        self.tolerance =  0.01 # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)       
 
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame =  '/base_footprint'

        # The odom frame is usually just /odom
        self.odom_frame = '/odom'

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(1)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        
            
        rospy.loginfo("move forward 0.15m")
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()
        
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()
        error = 1.0
        # while not rospy.is_shutdown():
        while abs(error) >  self.tolerance:
            # Stop the robot by default
            move_cmd = Twist()
            
            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                                
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction
                
                # How close are we?
                error =  distance - self.test_distance
                # rospy.loginfo(error)
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = -self.speed #copysign(self.speed, -1 * error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())


    def move_backward(self):
        r = rospy.Rate(self.rate)
        # Set the distance to travel
        self.test_distance =  0.15 # meters
        self.speed =  0.10 # meters per second
        self.tolerance = 0.01 # meters
        self.odom_linear_scale_correction = 1.0
        self.start_test = True
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame =  '/base_footprint'

        # The odom frame is usually just /odom
        self.odom_frame = '/odom'

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(1)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        
            
        rospy.loginfo("move backward 0.15m")
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()
        
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()

        error = 1.0      
        # while not rospy.is_shutdown():
        while abs(error) >  self.tolerance:
            # Stop the robot by default
            move_cmd = Twist()
            
            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                                
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction
                
                # How close are we?
                error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            # rospy.loginfo(move_cmd)
            

        # Stop the robot
        # rospy.loginfo("1111122222")
        self.cmd_vel.publish(Twist())
        



    def move_left(self):
        r = rospy.Rate(self.rate)
        
        # The test angle is 360 degrees
        self.test_angle = radians(10.0)

        self.speed = rospy.get_param('~speed', 0.5) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(1)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("move left 10 degrees")
        
        reverse = 1
        
        while not rospy.is_shutdown():
            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle
                                    
                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}
                # dyn_client.update_configuration(params)
                
            rospy.sleep(0.5)
            break
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())

    def move_right(self):
        r = rospy.Rate(self.rate)
        
        # The test angle is 360 degrees
        self.test_angle = radians(10.0)

        self.speed = rospy.get_param('~speed', 0.5) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(1)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("move right 10 degrees")
        
        reverse = -1
        
        while not rospy.is_shutdown():
            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle
                                    
                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}
                # dyn_client.update_configuration(params)
                
            rospy.sleep(0.5)
            break
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())       



    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        # Convert the rotation from a quaternion to an Euler angle
        return self.quat_to_angle(Quaternion(*rot))
            


    def quat_to_angle(self,quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self,angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 



 
if __name__ == '__main__':
    try:
       RandomMoveWithCamera()
    #    rospy.spin
    except:
        rospy.loginfo("Program terminated.")