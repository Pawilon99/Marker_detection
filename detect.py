import numpy as np
import cv2
import os
import argparse
import yaml
import pickle
from glob import glob
import cv2.aruco as aruco
import time
import zipfile


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate camera using a video of a chessboard or a sequence of images.')
    parser.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL",help="type of ArUCo tag to detect")
    parser.add_argument('out',nargs="?",help='output calibration yaml file')
    parser.add_argument('--output_dir',nargs="?",help='path to directory where calibration files will be saved.',default='/home/pi/Desktop/ArUco')
    args = parser.parse_args()

    matrix = open('/home/pi/Desktop/calib_data/cameraMatrix.txt', 'r')
    distortion = open('/home/pi/Desktop/calib_data/cameraDistortion.txt', 'r')
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11}
    

    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters_create()
 



    cap = cv2.VideoCapture(1)
    file1 = matrix.read()
    file2 = distortion.read()
    
    camera_width = 640
    camera_height = 480
    camera_frame_rate = 20

    cap.set(2, camera_width)
    cap.set(4, camera_height)
    cap.set(5, camera_frame_rate)
    
    
    ret, frame = cap.read()

    grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(grey_frame, arucoDict, file1, file2)

    def aruco_display(corners, ids, rejected, images):
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
        # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
