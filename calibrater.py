import cv2
import numpy as np
import glob
import os

def calibrate_camera():
    # Chessboard parameters - MUST MATCH YOUR CHECKERBOARD
    pattern_size = (7, 5)  # Number of INTERNAL corners (cols, rows)
    square_size = 0.0265    # Size of squares in meters (adjust to your printed size)
    
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., (7,5,0)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    # Load calibration images
    image_dir = 'calibration_images'
    if not os.path.exists(image_dir):
        print(f"Error: Directory '{image_dir}' not found!")
        return None, None
    
    images = glob.glob(f'{image_dir}/*.jpg') + glob.glob(f'{image_dir}/*.png')
    
    if not images:
        print(f"Error: No images found in '{image_dir}'!")
        return None, None
    
    print(f"Found {len(images)} images. Processing...")
    
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"Warning: Could not read image {fname}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        if ret:
            print(f"Found checkerboard in {fname}")
            objpoints.append(objp)
            
            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            
            # Visual feedback
            cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            cv2.imshow('Checkerboard', img)
            cv2.waitKey(500)
        else:
            print(f"Warning: Checkerboard not found in {fname}")
    
    cv2.destroyAllWindows()
    
    if not objpoints:
        print("Error: No valid checkerboard images found for calibration!")
        return None, None
    
    # Perform camera calibration
    print(f"\nCalibrating with {len(objpoints)} valid images...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("\nCalibration results:")
    print(f"RMS Re-projection Error: {ret}")
    print("\nCamera matrix:\n", mtx)
    print("\nDistortion coefficients:", dist.ravel())
    
    # Save calibration parameters
    np.savez('camera_params.npz', mtx=mtx, dist=dist)
    
    return mtx, dist

# Run calibration
mtx, dist = calibrate_camera()

if mtx is not None:
    print("\nCalibration successful! Parameters saved to camera_params.npz")
else:
    print("\nCalibration failed. Please check:")
    print("1. The 'calibration_images' directory exists and contains images")
    print("2. Your checkerboard pattern matches the pattern_size (internal corners)")
    print("3. The images clearly show the full checkerboard pattern")
