import cv2
import numpy as np
from pupil_apriltags import Detector

# Replace the camera matrix with your provided values
K = np.array([[1.51717086e+03, 0.00000000e+00, 1.00740748e+03],
              [0.00000000e+00, 1.52168781e+03, 5.40380046e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

def rotation_matrix_to_euler_angles(R):
    # Calculate Euler angles from a 3x3 rotation matrix R
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def triangulate_pose(tags, K):
    tag_size = 0.16  # Assuming a tag size of 16 cm
    tag_poses = []

    for tag in tags:
        # Extract tag ID, corners, and homography
        tag_id = tag.tag_id
        H = tag.homography

        # Triangulate the tag's pose
        _, Rs, Ts, _ = cv2.decomposeHomographyMat(H, K)
        R = Rs[0]  # Extract the first rotation matrix from Rs

        # Check if the determinant of each rotation matrix is greater than zero
        if np.all(np.linalg.det(R) > 0):
            # Calculate Euler angles from the rotation matrix
            euler_angles = rotation_matrix_to_euler_angles(R)
            yaw, pitch, roll = euler_angles

            tag_poses.append({
                'tag_id': tag_id,
                'translation': Ts[0],  # Extract the first translation vector from Ts
                'rotation': euler_angles,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll
            })

    return tag_poses

# Create an AprilTags detector
detector = Detector(families='tag36h11')

# Use cv2.CAP_DSHOW as the capture backend (adjust the camera index as needed)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    tags = detector.detect(gray)

    if tags:
        for tag in tags:
            # Draw the outline of the detected AprilTag
            for j in range(len(tag.corners)):
                cv2.line(frame, tuple(tag.corners[j - 1].astype(int)), tuple(tag.corners[j].astype(int)), (0, 255, 0), 2)

            # Triangulate the pose and draw coordinate axes
            tag_pose = triangulate_pose([tag], K)[0]

            # Print pose information
            print(f"Tag ID: {tag.tag_id}")
            print(f"Translation: {tag_pose['translation']}")
            print(f"Rotation (Yaw, Pitch, Roll): {tag_pose['yaw']}, {tag_pose['pitch']}, {tag_pose['roll']}")

            # Draw coordinate axes on the tag in 2D space
            axis_length = 50  # Length of the axis lines in pixels (adjust as needed)

            # Convert the translation vector to a 1D array
            translation_1d = np.array(tag_pose['translation'][:2]).flatten()

            # Define R_matrix (rotation matrix)
            R_matrix = cv2.Rodrigues(tag_pose['rotation'])[0]

            # Define T_2D (translation)
            T_2D = tuple(map(int, translation_1d))

            # Calculate 2D coordinates
            direction_x = np.dot(R_matrix[:, 0][:2], [1, 0])  # Calculate the direction vector along the X-axis
            direction_y = np.dot(R_matrix[:, 1][:2], [0, 1])  # Calculate the direction vector along the Y-axis

            p2_x_2D = tuple(map(int, (translation_1d + axis_length * direction_x)))
            p2_y_2D = tuple(map(int, (translation_1d + axis_length * direction_y)))

            # Draw the axes
            cv2.line(frame, T_2D, p2_x_2D, (0, 0, 255), 2)  # X-axis (Red)
            cv2.line(frame, T_2D, p2_y_2D, (0, 255, 0), 2)  # Y-axis (Green)

    cv2.imshow('AprilTags Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()