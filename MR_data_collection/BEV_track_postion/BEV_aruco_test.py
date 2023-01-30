import cv2
import numpy as np
# import cv2.aruco as aruco
from cv2 import aruco

#Aruco detection
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0)
# cap.set(3, 1920)  # width=1920
# cap.set(4, 1080)  # height=1080
cap.set(3,960)  # width=1920
cap.set(4, 960)  # height=1080

while True:
   ret, frame = cap.read()
   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

   aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
   arucoParameters = aruco.DetectorParameters_create()
   corners, ids, rejectedImgPoints = aruco.detectMarkers(
       gray, aruco_dict, parameters=arucoParameters)
   print(ids)
   if np.all(ids):
     image = aruco.drawDetectedMarkers(frame,corners,ids)         
     cv2.imshow('Display', image)
   else:
       display = frame
       cv2.imshow('Display', display)
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break

#    if np.all(ids):
#      image = aruco.drawDetectedMarkers(frame,corners,ids)         
#      #Resize the image to a lower resolution
#      width = 640
#      height = 480
#      dim = (width, height)
#      resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_LINEAR)
#      cv2.imshow('Display', resized_image)
#    else:
#        display = frame
#        #Resize the image to a lower resolution
#        width = 640
#        height = 480
#        dim = (width, height)
#        resized_image = cv2.resize(display, dim, interpolation = cv2.INTER_LINEAR)
#        cv2.imshow('Display', resized_image)         
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break

cap.release()
cv2.destroyAllWindows()