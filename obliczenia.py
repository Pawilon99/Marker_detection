import rospy
import numpy as np
import cv2
from my_project.msg import vectors

tvecs_rec = 0
rvecs_rec = 0
ids = 0
raspberry = 0
distance = 0
rvecs = 0
tvecs = 0
info1 = False
rotation_matrix = 0
point = 0
info2 = False

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    global tvecs_rec
    global rvecs_rec
    global ids
    global raspberry
    global distance
    global tvecs
    global rvecs
    global info
    global rotation_matrix
    global point

    tvecs_rec = [data.x, data.y, data.z]
    rvecs_rec = [data.i, data.j, data.k]
    tvecs = np.array(tvecs_rec)
    rvecs = np.array(rvecs_rec)
    #ids = [data.l]
    raspberry = [data.m]
    
    if info1==False and raspberry == 1.0:
        rotation_matrix = cv2.Rodrigues(rvecs)
        distance = np.sqrt(tvecs[0]**2 + tvecs[1]**2 + tvecs[2]**2)
        T = np.concatenate((rotation_matrix, tvecs), axis=1)
        T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
        T_inv = np.linalg.inv(T)
        point_camera = np.array([[0],[0], [distance]])
        new_point = np.vstack((point_camera, 1.0))
        point_desire = T_inv.dot(new_point)
        point = point_desire[:3]

        info = True
        
    if info2==False and raspberry == 2.0:
        rotation_matrix = cv2.Rodrigues(rvecs)
        distance = np.sqrt(tvecs[0]**2 + tvecs[1]**2 + tvecs[2]**2)
        T = np.concatenate((rotation_matrix, tvecs), axis=0)
        T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
        T_inv = np.linalg.inv(T)
        point_camera = np.array([[0],[0], [distance]])
        new_point = np.vstack((point_camera, 1.0))
        point_desire = T_inv.dot(new_point)
        point = point_desire[:3]

        info = True
    
    # if (raspberry == 1.0):
    #     distance = np.sqrt(tvecs[0]**2 + tvecs[1]**2 + tvecs[2]**2)
    # else:
    #     distance = np.sqrt(tvecs[0]**2 + tvecs[1]**2 + tvecs[2]**2)

    print(point)
    

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', vectors, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
