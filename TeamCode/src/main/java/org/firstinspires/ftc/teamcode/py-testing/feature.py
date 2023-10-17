import cv2
import numpy as np

# Define camera calibration parameters
camera_matrix = np.array([[1.91086092e+03, 0.00000000e+00, 6.30917488e+02],
                          [0.00000000e+00, 1.90780998e+03, 4.98104766e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([0.03201133, 0.43087633, -0.01753296, -0.01625355, -4.10677589])

# Initialize the camera (you may need to use a different library depending on your camera)
cap = cv2.VideoCapture(0)

# Define the known coordinates of a reference point on the field
reference_point = np.array([0, 0, 0])  # Replace with actual values

# Define a function to find and track features in the image and calculate robot coordinates
def track_and_calculate_coordinates(frame):
    # Undistort the frame based on camera calibration
    undistorted_frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)
    
    # Convert the frame to grayscale for feature tracking
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
    
    # Use a feature detection algorithm (e.g., FAST, ORB, etc.) to find key points
    detector = cv2.FastFeatureDetector_create()
    keypoints = detector.detect(gray, None)
    
    # Calculate the robot's position in the camera's coordinate system
    if len(keypoints) > 0:
        # For simplicity, assume the robot's position is the mean of the detected keypoints
        robot_x = np.mean([kp.pt[0] for kp in keypoints])
        robot_y = np.mean([kp.pt[1] for kp in keypoints])
        
        # Draw a circle at the robot's position
        cv2.circle(undistorted_frame, (int(robot_x), int(robot_y)), 10, (0, 255, 0), -1)
    else:
        robot_x, robot_y = -1, -1
    
    return undistorted_frame, robot_x, robot_y

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    
    # Process the frame to track features and calculate robot coordinates
    frame, robot_x, robot_y = track_and_calculate_coordinates(frame)
    
    # Convert robot coordinates from camera coordinate system to absolute coordinates
    if robot_x != -1 and robot_y != -1:
        # Assuming a simple transformation (you may need more complex calibration)
        absolute_x = robot_x + reference_point[0]
        absolute_y = robot_y + reference_point[1]
        
        # Print the absolute robot coordinates
        print(f"Absolute Robot Coordinates: ({absolute_x}, {absolute_y})")
        
        # Display the absolute coordinates on the frame
        cv2.putText(frame, f"X: {absolute_x:.2f} Y: {absolute_y:.2f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Display the undistorted frame with detected keypoints
    cv2.imshow('Robot Tracking', frame)
    
    # Exit the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()