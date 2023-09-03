import cv2
import math
import numpy as np
from pupil_apriltags import Detector

K = np.array([[1.51717086e+03, 0.00000000e+00, 1.00740748e+03],
              [0.00000000e+00, 1.52168781e+03, 5.40380046e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

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
    tag_size = 0.16
    tag_poses = []

    for tag in tags:
        tag_id = tag.tag_id
        H = tag.homography

        _, Rs, Ts, _ = cv2.decomposeHomographyMat(H, K)
        R = Rs[0]

        if np.all(np.linalg.det(R) > 0):
            euler_angles = rotation_matrix_to_euler_angles(R)
            yaw, pitch, roll = euler_angles

            tag_poses.append({
                'tag_id': tag_id,
                'translation': Ts[0],
                'rotation': euler_angles,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll
            })

    return tag_poses

detector = Detector(families='tag36h11')
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)

    if tags:
        for tag in tags:
            for j in range(len(tag.corners)):
                cv2.line(frame, tuple(tag.corners[j - 1].astype(int)), tuple(tag.corners[j].astype(int)), (0, 255, 0), 2)

            tag_pose = triangulate_pose([tag], K)[0]

            yaw_degrees = math.degrees(tag_pose['yaw'])
            pitch_degrees = math.degrees(tag_pose['pitch'])
            roll_degrees = math.degrees(tag_pose['roll'])

            axis_length = 50

            corners = tag.corners.astype(int)
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))

            rotation_matrix = cv2.Rodrigues(tag_pose['rotation'])[0]
            x_direction = np.dot(rotation_matrix, np.array([axis_length, 0, 0]))
            y_direction = np.dot(rotation_matrix, np.array([0, axis_length, 0]))
            z_direction = np.dot(rotation_matrix, np.array([0, 0, axis_length]))

            cv2.line(frame, (center_x, center_y), (int(center_x + x_direction[0]), int(center_y + x_direction[1])), (0, 0, 255), 2)
            cv2.line(frame, (center_x, center_y), (int(center_x + y_direction[0]), int(center_y + y_direction[1])), (0, 255, 0), 2)
            cv2.line(frame, (center_x, center_y), (int(center_x + z_direction[0]), int(center_y + z_direction[1])), (255, 0, 0), 2)

            rotation_text = f"Yaw: {yaw_degrees:.2f}, Pitch: {pitch_degrees:.2f}, Roll: {roll_degrees:.2f}"
            cv2.putText(frame, rotation_text, (center_x - 80, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.imshow('Visual Localization', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
