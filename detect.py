import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import json

# Dekalaracja parametrów ArUco
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters_create()

# Deklaracja parametrów kamery

with open('/home/asus/parameters.json','r') as parameters:
    camera_params = json.load(parameters)
    parameters.close()

camera_matrix = camera_params['camera_matrix']
camera_distortion = camera_params['camera_distortion']



# Zainicjalizowanie działania kamery
cap = cv2.VideoCapture(0) 

if __name__ == '__main__':
    rospy.init_node('publisher', anonymous=True)
    pub = rospy.Publisher('chatter', Float64 , queue_size=10)
    rate = rospy.Rate(10)
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
            # Oszacowanie pozycji markerów
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, camera_distortion)

            # Wyrysowanie pozcyji markerów na obrazie
            for i in range(len(ids)):

                distance = np.sqrt(tvecs[i][0][2]**2 + tvecs[i][0][0]**2 + tvecs[i][0][1]**2)
                cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvecs[i], tvecs[i], 0.01)
                aruco.drawDetectedMarkers(frame, corners)
#                 cv2.putText(
#                 frame, 
#                 f"id: {ids} Dist: {round(distance*100,2)}",
#                 (10,50),
#                 cv2.FONT_HERSHEY_PLAIN,
#                 1, 
#                 (200,100,255), 
#                 2, 
#                 cv2.LINE_AA)

#                 cv2.putText(
#                 frame, 
#                 f"id: {ids} x: {round((tvecs[i][0][0])*100,1)} y: {round((tvecs[i][0][1])*100,1)}",
#                 (10,100),
#                 cv2.FONT_HERSHEY_PLAIN,
#                 1, 
#                 (200,100,255), 
#                 2, 
#                 cv2.LINE_AA)


            # Print marker IDs and poses
            print("Marker IDs: ", ids)
            print("Translation vectors: ", tvecs)
            print("Rotation vectors: ", rvecs)
            list_param = [ids, tvecs, rvecs, distance*100]
            rospy.loginfo(list_param)
            pub.publish(list_param)
            
            
    # Display frame with marker poses
    # cv2.imshow('Distributed Object Positioning System', frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
rospy.spin()
# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
