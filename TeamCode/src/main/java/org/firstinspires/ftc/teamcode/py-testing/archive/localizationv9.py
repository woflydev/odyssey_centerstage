import cv2
import math
import numpy as np
import configparser
from pupil_apriltags import Detector

# Replace the camera matrix with your provided values
K = np.array([[1.51717086e+03, 0.00000000e+00, 1.00740748e+03],
              [0.00000000e+00, 1.52168781e+03, 5.40380046e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

axis_length = 50 # in pixels
tag_size = 0.168 # in metres
camera_fov = 90 # in degrees

def create_ema_filter(alpha=0.035):
    """Create an Exponential Moving Average (EMA) filter with a given smoothing factor (alpha)."""
    filtered_value = None

    def filter_value(new_value):
        nonlocal filtered_value
        if filtered_value is None:
            filtered_value = new_value
        else:
            filtered_value = alpha * new_value + (1 - alpha) * filtered_value
        return filtered_value[0]

    return filter_value

yaw_filter = create_ema_filter()
pitch_filter = create_ema_filter()
roll_filter = create_ema_filter()

def calculate_robot_position(tag_distance, camera_angle, camera_fov, tag_x):
    angle_to_tag = (camera_fov / 2) - camera_angle
    x_offset = tag_distance * math.cos(math.radians(angle_to_tag)) - 330 # scalar - not sure why it's scuffed
    robot_x = tag_x + x_offset

    return robot_x / 1000 # convert to metres

def draw_text(frame, text, position, font_scale=0.6, color=(0, 255, 255), thickness=2, font=cv2.FONT_HERSHEY_SIMPLEX, line_type=2):
    text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
    line_height = text_size[1] + 5
    x, y0 = position
    for i, line in enumerate(text.split("\n")):
        y = y0 + i * line_height
        cv2.putText(frame,
                    line,
                    (x, y),
                    font,
                    font_scale,
                    color,
                    thickness,
                    line_type)

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

            # Calculate the distance to the tag using the known tag size and the detected tag size in pixels
            tag_size_pixels = max(tag.corners[:, 0]) - min(tag.corners[:, 0])
            distance = (tag_size * K[0, 0]) / (2 * tag_size_pixels) # convert to mm

            tag_poses.append({
                'tag_id': tag_id,
                'translation': Ts[0],  # Extract the first translation vector from Ts
                'rotation': euler_angles,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll,
                'distance': distance  # Add distance to the tag's pose
            })

    return tag_poses

# Create an AprilTags detector
detector = Detector(families='tag36h11')

# Use cv2.CAP_DSHOW as the capture backend (adjust the camera index as needed)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

config = configparser.ConfigParser()

try:
    config.read('py_testing_config.ini')
    if 'Calibration' in config:
        # Load the calibration_offset values if they exist in the configuration
        calibration_offset_x = float(config['Calibration'].get('calibration_offset_x', 0.0))
        calibration_offset_y = float(config['Calibration'].get('calibration_offset_y', 0.0))
        calibration_offset_z = float(config['Calibration'].get('calibration_offset_z', 0.0))
        distance_offset = float(config['Calibration'].get('distance_offset', 0.0))
    else:
        calibration_offset_x = 0.0
        calibration_offset_y = 0.0
        calibration_offset_z = 0.0
        distance_offset = 0.0
except Exception as e:
    print(f"Error reading config: {e}")
    calibration_offset_x = 0.0
    calibration_offset_y = 0.0
    calibration_offset_z = 0.0
    distance_offset = 0.0

calibration_offset = np.array([calibration_offset_x, calibration_offset_y, calibration_offset_z])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    tags = detector.detect(gray)

    if tags:
        closest_tag = None
        min_distance = float('inf')
        for tag in tags:
            distance = triangulate_pose([tag], K)[0]['distance']
            if distance < min_distance:
                min_distance = distance
                closest_tag = tag

        for tag in tags:
            # Draw the outline of the detected AprilTag
            for j in range(len(tag.corners)):
                cv2.line(frame, tuple(tag.corners[j - 1].astype(int)), tuple(tag.corners[j].astype(int)), (0, 255, 0), 2)

            # Triangulate the pose and draw coordinate axes
            tag_pose = triangulate_pose([tag], K)[0]
            closest_tag_pose = triangulate_pose([closest_tag], K)[0]

            # Add the calibration offset to the tag's rotation
            tag_pose['rotation'] += calibration_offset
            tag_pose['distance'] += distance_offset

            # Extract the tag's corners
            corners = tag.corners.astype(int)
            closest_corners = closest_tag.corners.astype(int)

            # Calculate the center of the tag
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))
            closest_center_x = int(np.mean(closest_corners[:, 0]))

            # Calculate the direction vectors based on the tag's orientation
            rotation_matrix = cv2.Rodrigues(tag_pose['rotation'])[0]
            x_direction = np.dot(rotation_matrix, np.array([axis_length, 0, 0]))
            y_direction = np.dot(rotation_matrix, np.array([0, axis_length, 0]))
            z_direction = np.dot(rotation_matrix, np.array([0, 0, axis_length]))

            # Draw the axes relative to the camera
            cv2.line(frame, (center_x, center_y), (int(center_x + x_direction[0]), int(center_y + x_direction[1])), (0, 0, 255), 2)  # X-axis (Red)
            cv2.line(frame, (center_x, center_y), (int(center_x + y_direction[0]), int(center_y + y_direction[1])), (0, 255, 0), 2)  # Y-axis (Green)
            cv2.line(frame, (center_x, center_y), (int(center_x + z_direction[0]), int(center_y + z_direction[1])), (255, 0, 0), 2)  # Z-axis (Blue)

            filtered_yaw = yaw_filter(rotation_matrix[0])
            filtered_pitch = pitch_filter(rotation_matrix[1])
            filtered_roll = roll_filter(rotation_matrix[2])

            yaw_deg, pitch_deg, roll_deg = np.rad2deg([filtered_yaw, filtered_pitch, filtered_roll])

            # Calculate robot's X and Z coordinates
            robot_x = calculate_robot_position(closest_tag_pose['distance'], closest_tag_pose['yaw'], camera_fov, closest_center_x)
            robot_z = closest_tag_pose['distance'] # distance is already calculated

            # Display rotation and distance information as text in the top-right corner of the AprilTag
            using_id_text = f"Using Closest ID: {closest_tag_pose['tag_id']}"
            robot_coordinates_text = f"Robot X: {robot_x:.2f}, Robot Z: {robot_z:.2f}"
            id_text = f"ID: {tag_pose['tag_id']}"
            info_text = f"Yaw: {yaw_deg:.0f}\nPitch: {pitch_deg:.0f}\nRoll: {roll_deg:.0f}\nDistance: {tag_pose['distance']:.2f} meters"
            draw_text(frame, using_id_text, (10, 30), thickness=2, color=(255, 255, 0))
            draw_text(frame, robot_coordinates_text, (10, 60), thickness=2, color=(0, 0, 255))
            draw_text(frame, id_text, (corners[1][0], corners[1][1] - 60), color=(255, 255, 0))
            draw_text(frame, info_text, (corners[1][0], corners[1][1] - 30))

    calibration_text = f"Calibration Offset:\nYaw: {calibration_offset[0]:.1f}\nPitch: {calibration_offset[1]:.1f}\nRoll: {calibration_offset[2]:.1f}\nDistance: {distance_offset:.2f} meters"
    draw_text(frame, calibration_text, (10, 90), thickness=2, color=(255, 255, 255))

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
    elif key == ord('z'):
        distance_offset += 0.01
    elif key == ord('x'):
        distance_offset -= 0.01
    elif key == ord('c'):
        distance_offset = 0.0


cap.release()
cv2.destroyAllWindows()

try:
    if 'Calibration' not in config:
        config.add_section('Calibration')
    config.set('Calibration', 'calibration_offset_x', str(calibration_offset[0]))
    config.set('Calibration', 'calibration_offset_y', str(calibration_offset[1]))
    config.set('Calibration', 'calibration_offset_z', str(calibration_offset[2]))
    config.set('Calibration', 'distance_offset', str(distance_offset))
    
    # Write the updated configuration back to the file
    with open('py_testing_config.ini', 'w') as configfile:
        config.write(configfile)
except Exception as e:
    print(f"Error writing config: {e}")