import rospy
import numpy as np
import cv2
from my_project.msg import vectors
import json

tvecs_rec = 0
rvecs_rec = 0
ids = 0
raspberry = 0
distance = 0
info1 = False
info2 = False
rotation_matrix = 0
point = 0
coordinates = 0
T1 = 0
T1_inv = 0
T2 = 0
T2_inv = 0
vector = 0
j = {}

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    global tvecs_rec
    global rvecs_rec
    global ids
    global raspberry
    global info1
    global info2
    global rotation_matrix
    global point
    global coordinates
    global T1
    global T1_inv
    global T2
    global T2_inv
    global vector
    global j
    
    tvecs_rec = np.array([[data.x], [data.y], [data.z]])
    rvecs_rec = np.array([[data.i], [data.j], [data.k]])
    raspberry = data.m
    
    if raspberry == 1.0 and info1 == False:
        rotation_matrix, _ = cv2.Rodrigues(rvecs_rec)
        T1 = np.concatenate((rotation_matrix, tvecs_rec), axis=1)
        T1 = np.concatenate((T1, np.array([[0, 0, 0, 1]])), axis=0)
        T1_inv = np.linalg.inv(T1)
    
        info1 = True

    if raspberry == 2.0 and info2 == False:
        rotation_matrix, _ = cv2.Rodrigues(rvecs_rec)
        T2 = np.concatenate((rotation_matrix, tvecs_rec), axis=1)
        T2 = np.concatenate((T2, np.array([[0, 0, 0, 1]])), axis=0)
        T2_inv = np.linalg.inv(T2)

        info2 = True

    if (raspberry == 1.0):
        vector = np.array([[0],[0],[0],[1]])
        rotation_matrix, _ = cv2.Rodrigues(rvecs_rec)
        T1 = np.concatenate((rotation_matrix, tvecs_rec), axis=1)
        T1 = np.concatenate((T1, np.array([[0, 0, 0, 1]])), axis=0)
        point = T1_inv@T1@vector
        j['coordinates'] = point.tolist()
        coordinates = point 
    else:
        vector = np.array([[0],[0],[0],[1]])
        rotation_matrix, _ = cv2.Rodrigues(rvecs_rec)
        T2 = np.concatenate((rotation_matrix, tvecs_rec), axis=1)
        T2 = np.concatenate((T2, np.array([[0, 0, 0, 1]])), axis=0)
        point = T2_inv@T2@vector
        j['coordinates1'] = point.tolist()
        coordinates = point
        
    print(coordinates)  
    
    
    with open('/home/asus/catkin_ws/src/my_project/scripts/coordinates.json', 'w') as jsonFile:
        json.dump(j,jsonFile)
        jsonFile.close()

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', vectors, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
