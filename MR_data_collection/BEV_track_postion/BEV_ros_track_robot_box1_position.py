
import cv2
import numpy as np
import math
from cv2 import aruco
import time

import rospy
from turtlesim.msg import Pose

rospy.init_node('camera_perspective', anonymous=True)
pose_publisher = rospy.Publisher('/turtle1/pose', Pose, queue_size=10)
box1_publisher = rospy.Publisher('/turtle1/box1',Pose,queue_size=10)


robot_pose = Pose()
box1_pose = Pose()


# modify these based on your enviornment settings
robot_ID = 6
box1_ID =7

# bottom_left = 1  #this is the origin - positivex: towards bottom right - positivey: towards top left
# bottom_right = 4
# top_left = 2
# top_right = 3

bottom_left = 1  #this is the origin - positivex: towards bottom right - positivey: towards top left
bottom_right = 2
top_right = 3
top_left = 4


marker_dimension = 0.046  #4.6 centimeter before
# worldx = 889 #508#-marker_dimension*1000 #millimeters
# worldy = 508 #401#-marker_dimension*1000 #millimeters
worldx = 1500#508#-marker_dimension*1000 #millimeters length and width of the area
worldy = 1500 #401#-marker_dimension*1000 #millimeters


def getMarkerCenter(corners):
 px = (corners[0][0] + corners[1][0] + corners[2][0]+ corners[3][0]) * 0.25
 py = (corners[0][1] + corners[1][1] + corners[2][1]+ corners[3][1]) * 0.25
 return [px,py]

def getMarkerRotation(corners):
 unit_x_axis = [1.,0.]
 center = getMarkerCenter(corners)
 right_edge_midpoint = (corners[0]+corners[1])/2.
 unit_vec = (right_edge_midpoint-center)/np.linalg.norm(right_edge_midpoint-center)
#  angle = np.arccos(np.dot(unit_x_axis,unit_vec))
 angle = math.atan2(right_edge_midpoint[1] - center[1], right_edge_midpoint[0] - center[0])
 return angle

def inversePerspective(rvec, tvec):
   R, _ = cv2.Rodrigues(rvec)
   R = np.array(R).T #this was np.matrix but had error
   invTvec = np.dot(-R, np.array(tvec))
   invRvec, _ = cv2.Rodrigues(R)
   return invRvec, invTvec
def normalize(v):
   if np.linalg.norm(v) == 0 : return v
   return v / np.linalg.norm(v)

"""" the function gets the corners of an aruco marker in the camera space as the origin with its
X (Green) and Y (Red) axis. The origin is the bottom left corner, the coordinate of the point is calculated
in relation to this origin """
def findWorldCoordinate(originCorners, point):
   zero = np.array(originCorners[3]) #bottom left as the origin - check the data structure
   x = (np.array(originCorners[0]) - zero)  # bottom right - Green Axis -- throw out z
   y = (np.array(originCorners[1]) - zero)   # top left - Red Axis -- throw out z
   x = x[0][0:2]
   y = y[0][0:2]
   x = normalize(x)
   y = normalize(y)
   #print("x", x)
   vec = (point - zero)[0][0:2]
   #print("vec", vec)
   vecNormal = normalize(vec)
   cosX = np.dot(x,vecNormal)
   cosY = np.dot(y,vecNormal)
   xW = np.linalg.norm(vec) * cosX
   yW = np.linalg.norm(vec) * cosY
   return [xW, yW]
################################
try:
   # Getting the camera calibration information
#    path = r'/home/pi/ECE-5725/camera_codes/picamera.yml'
#    path = r'F:\System Folders\Desktop\RP_reference\RP_HW\project221020\picamera.yml' #still need to calibrate
   path = r'picamera_1.yml' #still need to calibrate 
   cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
   new_matrix = cv_file.getNode("new_matrix").mat()
   dist_matrix = cv_file.getNode("distortion_coef").mat()
   mtx = cv_file.getNode("matrix").mat()
   cv_file.release()

   found_dict_pixel_space = {}
   found_dict_camera_space = {}
   found_dict_world_space = {}
   found_dict_homography_space = {}
   final_string = ""
   originRvec = np.array([0,0,1])
   markerRvec= np.array([0,0,0])

 # read aruco markers and create dictionary
#    aruco_source_path = glob.glob(r'/home/pi/ECE-5725/camera_codes/aruco/*.jpg', )
#    aruco_source = []
#    for ar in aruco_source_path:
#        im_src = cv2.imread(ar)
#        aruco_source.append(im_src)
   aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
  
   cap = cv2.VideoCapture(0)
   # cap.set(3, 1920)  # width=1920
   # cap.set(4, 1080)  # height=1080
   #pool = mp.Pool(mp.cpu_count()) # init pool for multiprocessing
  
   while True:
       ret, image = cap.read() # read frame
       #----aruco detection-------------------
       gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # converts image color space from BRG to grayscale
       data = aruco.detectMarkers(gray, aruco_dict) # detect aruco markers
      
       corners = data[0] # corners of found aruco markers
       ids = data[1] # ids of found aruco markers #array([[4],[1],[3],[2],[6]], dtype=int32)
       originIDglobal = 0
       if np.all(ids): # if any markers were found
           #print("found one!")
           result = aruco.estimatePoseSingleMarkers(corners, marker_dimension, new_matrix, dist_matrix) # estimate the pose of the markers
           rvecs = result[0] # rotation vectors of markers
           tvecs = result[1] # translation vector of markers
           imageCopy = image

           #setting bottom_left as the origin
           if bottom_left in ids:
               originID = np.where(ids == bottom_left)[0][0]
               originIDglobal = originID
           else:
               originID = originIDglobal
           originCorners = corners[originID] # corners of the tag set as the origin
        #    OriginCornersCamera = getCornerInCameraWorld(marker_dimension, rvecs[originID], tvecs[originID])[0] # origin tag corners in camera space
           originRvec = rvecs[originID] # rotation vec of origin tag
           originTvec = tvecs[originID] # translation vec of origin tag
          
           display = cv2.aruco.drawDetectedMarkers(imageCopy,corners,ids) # display is image copy with boxes drawn around the tags
           for i in range(len(ids)): # for each tag found in image
               ID = ids[i]
               rvec = rvecs[i]
               tvec = tvecs[i]
               corners4 = corners[i]
              
               display = cv2.aruco.drawAxis(imageCopy,new_matrix,dist_matrix,rvec,tvec,0.03) # draw 3d axis, 3 centimeters
               found_dict_pixel_space[""+str(ids[i][0])] = corners4 # put the corners of this tag in the dictionary
          
           # Homography
           zero = found_dict_pixel_space[str(bottom_left)][0][3] #bottom left - 3
           x = found_dict_pixel_space[str(bottom_right)][0][2] #bottom right - 27
           y = found_dict_pixel_space[str(top_left)][0][0] #top left - 22
           xy = found_dict_pixel_space[str(top_right)][0][1] #top right - 24
         #   zero = found_dict_pixel_space[str(bottom_left)][0][0] #bottom left - 1
         #   x = found_dict_pixel_space[str(bottom_right)][0][3] #bottom right - 4
         #   y = found_dict_pixel_space[str(top_left)][0][1] #top left - 2
         #   xy = found_dict_pixel_space[str(top_right)][0][2] #top right - 3

           workspace_world_corners = np.array([[0.0, 0.0], [worldx, 0.0], [0.0, worldy], [worldx, worldy]], np.float32) # 4 corners in millimeters
           workspace_pixel_corners = np.array([zero,x,y,xy], np.float32)  # 4 corners in pixels

           # Homography Matrix
           h, status = cv2.findHomography(workspace_pixel_corners, workspace_world_corners) #perspective matrix
           im_out = cv2.warpPerspective(image, h, (worldx,worldy)) #showing that it works
           im_out= cv2.flip(im_out, 0)
           
           for i in range(len(ids)):
               j = ids[i][0]
               corners_pix = found_dict_pixel_space[str(j)]#[0]
               corners_pix_transformed = cv2.perspectiveTransform(corners_pix,h)
               found_dict_homography_space[str(j)] = corners_pix_transformed
           print(found_dict_homography_space)
           robot = found_dict_homography_space[str(robot_ID)][0]
           box1 = found_dict_homography_space[str(box1_ID)][0] #the 4 corners of box1_ID

           print(getMarkerCenter(robot))
           im_out = cv2.resize(im_out, (500,500), interpolation = cv2.INTER_LINEAR)
           cv2.imshow('Warped Source Image', im_out)

           cv2.imshow('Display', display)
      
       else:
           display = image
           cv2.imshow('Display', display)

       ###### sending the data to the robot ########################
       robot_center = getMarkerCenter(robot)
       robot_angle = getMarkerRotation(robot)
       time.sleep(0.1)

       #sending the robot's position and goal with publisher
    #    robot_pose.x = getMarkerCenter(robot_)[0]
    #    robot_pose.y = getMarkerCenter(robot_)[1]
    #    robot_pose.theta = getMarkerRotation(robot_)
    #    pose_publisher.publish(robot_pose)
       
      #  robot_pose.x = round(robot_center[0], 1) # round the value of 'robot_center[0]' to 4 decimal places
      #  robot_pose.y = round(robot_center[1], 1)
       
       robot_pose.x =robot_center[0]# round the value of 'robot_center[0]' to 4 decimqal places
       robot_pose.y = robot_center[1]
       print("robot_center:",robot_pose.x,robot_pose.y)
       robot_pose.theta = robot_angle
       pose_publisher.publish(robot_pose)

       box1_pose.x = round(getMarkerCenter(box1)[0], 1) # round the value of 'robot_center[0]' to 4 decimal places
       box1_pose.y = round(getMarkerCenter(box1)[1], 1) # doesn't work in rqt?
       print("box1_pose:",box1_pose.x,box1_pose.y)
      #  box1_pose.x = getMarkerCenter(box1)[0]
      #  box1_pose.y = getMarkerCenter(box1)[1]
      #  box1_pose.theta = getMarkerRotation(box1)
       box1_publisher.publish(box1_pose)



       #UDP(IP, port, str([robot_center[0], robot_center[1],robot_angle,worldx,worldy]))
    #    UDP(IP, port, str([robot_center[0], robot_center[1], robot[1][0], robot[1][1], robot[0][0], robot[0][1], worldx, worldy]))

       if cv2.waitKey(1) & 0xFF == ord('q'):
           break

except Exception as e:
 print(e)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()