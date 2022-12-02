#!/usr/bin/env python
# use to detect the aruco 
import cv2
import numpy as np
# import cv2.aruco as aruco
from cv2 import aruco

#Aruco detection
# cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(4)

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

cap.release()
cv2.destroyAllWindows()