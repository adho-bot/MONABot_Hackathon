import cv2
import os

def capture_checkerboard_images():
    # Create directory for saved images if it doesn't exist
    if not os.path.exists('calibration_images'):
        os.makedirs('calibration_images')
    
    # Find the next available image number
    n = 1
    while os.path.exists(f'calibration_images/checker_board_{n}.jpg'):
        n += 1
    
    # Initialize video capture
    cap = cv2.VideoCapture("rtsp://10.206.157.80:8554/hack")
    
    if not cap.isOpened():
        print("Error: Could not open video stream")
        return
    
    print("Press 's' to save image, 'q' to quit")
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Couldn't read frame")
            break
        
        # Display the frame
        cv2.imshow('Checkerboard Capture', frame)
        
        # Wait for key press
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s'):  # Save image
            img_path = f'calibration_images/checker_board_{n}.jpg'
            cv2.imwrite(img_path, frame)
            print(f"Saved {img_path}")
            n += 1
        elif key == ord('q'):  # Quit
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    print("Capture session ended")

if __name__ == "__main__":
    capture_checkerboard_images()
