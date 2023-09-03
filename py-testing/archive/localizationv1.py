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
        corners = tag.corners
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

# Capture video from the camera (adjust the camera index as needed)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    tags = detector.detect(gray)

    if tags:
        poses = triangulate_pose(tags, K)
        for pose in poses:
            print(f"Tag ID: {pose['tag_id']}")
            print(f"Translation: {pose['translation']}")
            print(f"Rotation (Yaw, Pitch, Roll): {pose['yaw']}, {pose['pitch']}, {pose['roll']}\n")

    cv2.imshow('AprilTags Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
