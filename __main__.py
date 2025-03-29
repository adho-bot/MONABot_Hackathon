import cv2 as cv
import tkinter as tk
from tkinter import simpledialog
import numpy as np

reference = cv.imread('images/ref.jpg',cv.IMREAD_GRAYSCALE) # trainImage

# Construct the RTSP URL
rtsp_url = f"rtsp://10.206.157.80:8554/hack"
print(f"Connecting to RTSP stream at: {rtsp_url}")

# Open the RTSP stream using OpenCV
cap = cv.VideoCapture(rtsp_url)
if not cap.isOpened():
    print("Error: Cannot open RTSP stream. Check the URL and network connectivity.")
    exit(1)

# Create a resizable OpenCV window
window_name = "Client - RTSP Stream"
cv.namedWindow(window_name, cv.WINDOW_NORMAL)  # Allow resizing
cv.resizeWindow(window_name, 800, 600)  # Set initial size

orb = cv.ORB_create()
bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
parameters = cv.aruco.DetectorParameters()

detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to retrieve frame from RTSP stream")
        break
    
    #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #corners, ids, rejected = detector.detectMarkers(gray)

    #if ids is not None:
    #    cv.aruco.drawDetectedMarkers(frame, corners, ids)

    #processed = cv.adaptiveThreshold(gray, 255, 
     #                   cv.ADAPTIVE_THRESH_GAUSSIAN_C, 
      #                  cv.THRESH_BINARY, 21, 20)
    
    cv.imshow(window_name, frame)
    # Exit if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        break

# Cleanup resources
cap.release()
cv.destroyAllWindows()
