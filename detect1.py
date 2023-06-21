#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import json
from my_project.msg import vectors

# Dekalaracja parametrów ArUco
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters_create()

# Deklaracja parametrów kamery

with open('/home/asus/catkin_ws/src/my_project/scripts/parameters1.json','r') as parameters:
    camera_params = json.load(parameters)
    parameters.close()

camera_matrix = camera_params['camera_matrix']
camera_distortion = camera_params['camera_distortion']



# Zainicjalizowanie działania kamery
cap = cv2.VideoCapture(2) 

if __name__ == '__main__':
    rospy.init_node('publisher', anonymous=True)
    pub = rospy.Publisher('chatter', vectors, queue_size=0.001)
    rate = rospy.Rate(0.001)
    list_param = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Przekształcenie obrazu w czarno-biały
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Wychwycenie znaczników
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if ids is not None:

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.04, np.float32(camera_matrix), np.float32(camera_distortion))
        
       
        print("Marker IDs: ", ids)
        print("Translation vectors: ", tvecs)
        print("Rotation vectors: ", rvecs)
        raspberry = 2.0
        tvecs1 = tvecs
        tvecs2 = tvecs1[0]
        tvecs_msg = tvecs2[0]
        rvecs1 = rvecs
        rvecs2 = rvecs1[0]
        rvecs_msg = rvecs2[0]
        msg = vectors()
        msg.x = tvecs_msg[0]
        msg.y = tvecs_msg[1]
        msg.z = tvecs_msg[2]
        msg.i = rvecs_msg[0]
        msg.j = rvecs_msg[1]
        msg.k = rvecs_msg[2]
        #msg.l = ids
        msg.m = raspberry    

        rospy.loginfo(msg)
        pub.publish(msg)
                
                
            
            
    
    rospy.spin()
# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
