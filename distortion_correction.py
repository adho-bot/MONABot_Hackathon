import cv2
import numpy as np

def setup_camera():
    cap = cv2.VideoCapture("rtsp://10.206.157.80:8554/hack")
    
    # Load calibration data
    with np.load('camera_params.npz') as data:
        mtx, dist = data['mtx'], data['dist']
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Undistort the frame
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        
        # Crop the image
        x, y, w, h = roi
        undistorted = undistorted[y:y+h, x:x+w]
        
        # Display results
        cv2.imshow('Original', frame)
        cv2.imshow('Undistorted', undistorted)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

setup_camera()
