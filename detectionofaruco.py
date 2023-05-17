import cv2
import cv2.aruco as aruco
import numpy as np

# Set ArUco marker parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters_create()

# Set camera parameters
camera_matrix = np.array([[6.263337898737789828e+02, 0, 3.061868201686978068e+02],
                          [0, 6.235291883889143492e+02, 2.248688353569037019e+02],
                          [0, 0, 1.000000000000000000e+00]])
camera_distortion = np.array([1.555969813511751954e-01, -1.275579002795265371e+00, -4.695900431116478702e-03, -5.325875938477707102e-04, 4.115031727676378814e+00])

# Capture video from camera
cap = cv2.VideoCapture(0)  # Use camera index 0, change to the appropriate index if using multiple cameras

while True:
    # Capture frame from camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    if ids is not None:
        # Estimate marker poses
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, camera_distortion)

        # Draw marker poses on frame
        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvecs[i], tvecs[i], 0.01)
            aruco.drawDetectedMarkers(frame, corners)
            cv2.putText(frame, 'Dist: tvecs[i][0][0]
            
            
        # Print marker IDs and poses
        print("Marker IDs: ", ids)
        print("Translation vectors: ", tvecs)
        print("Rotation vectors: ", rvecs)

    # Display frame with marker poses
    cv2.imshow('Distributed Object Positioning System', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
