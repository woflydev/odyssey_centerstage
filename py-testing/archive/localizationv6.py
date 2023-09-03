import cv2
import math
import numpy as np
from pupil_apriltags import Detector

# Replace the camera matrix with your provided values
K = np.array([[1.51717086e+03, 0.00000000e+00, 1.00740748e+03],
              [0.00000000e+00, 1.52168781e+03, 5.40380046e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

axis_length = 50  # Length of the axis lines in pixels (adjust as needed)

def rotation_matrix_to_euler_angles(R):
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

# Global variable to track calibration offset
calibration_offset = np.array([0.0, 0.0, 0.0])  # Initialize with zeros

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

            # Add the calibration offset to the tag's rotation
            tag_pose['rotation'] += calibration_offset

            # Extract the tag's corners
            corners = tag.corners.astype(int)

            # Calculate the center of the tag
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))

            # Calculate the direction vectors based on the tag's orientation
            rotation_matrix = cv2.Rodrigues(tag_pose['rotation'])[0]
            x_direction = np.dot(rotation_matrix, np.array([axis_length, 0, 0]))
            y_direction = np.dot(rotation_matrix, np.array([0, axis_length, 0]))
            z_direction = np.dot(rotation_matrix, np.array([0, 0, axis_length]))

            # Draw the axes relative to the camera
            cv2.line(frame, (center_x, center_y), (int(center_x + x_direction[0]), int(center_y + x_direction[1])), (0, 0, 255), 2)  # X-axis (Red)
            cv2.line(frame, (center_x, center_y), (int(center_x + y_direction[0]), int(center_y + y_direction[1])), (0, 255, 0), 2)  # Y-axis (Green)
            cv2.line(frame, (center_x, center_y), (int(center_x + z_direction[0]), int(center_y + z_direction[1])), (255, 0, 0), 2)  # Z-axis (Blue)

            yaw_deg, pitch_deg, roll_deg = np.rad2deg(tag_pose['rotation']) * -1

            # Display rotation information as text in the top-right corner of the AprilTag
            info_text = f"ID: {tag_pose['tag_id']}\nYaw: {yaw_deg:.1f}\nPitch: {pitch_deg:.1f}\nRoll: {roll_deg:.1f}"

            position = (corners[1][0], corners[1][1] - 30)
            font_scale = 0.6
            color = (0, 255, 255)
            thickness = 2
            font = cv2.FONT_HERSHEY_SIMPLEX
            line_type = 2

            text_size, _ = cv2.getTextSize(info_text, font, font_scale, thickness)
            line_height = text_size[1] + 5
            x, y0 = position
            for i, line in enumerate(info_text.split("\n")):
                y = y0 + i * line_height
                cv2.putText(frame,
                            line,
                            (x, y),
                            font,
                            font_scale,
                            color,
                            thickness,
                            line_type)

    cv2.imshow('Visual Localization', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('\\'):
        break

    # adjustments are in radians
    if key == ord('w'):
        calibration_offset[0] += 0.1
    elif key == ord('s'):
        calibration_offset[0] -= 0.1
    elif key == ord('a'):
        calibration_offset[1] -= 0.1
    elif key == ord('d'):
        calibration_offset[1] += 0.1
    elif key == ord('q'):
        calibration_offset[2] += 0.1
    elif key == ord('e'):
        calibration_offset[2] -= 0.1
    elif key == ord('r'):
        calibration_offset = -tag_pose['rotation'] if (not np.any(calibration_offset)) else np.array([0.0, 0.0, 0.0])

cap.release()
cv2.destroyAllWindows()