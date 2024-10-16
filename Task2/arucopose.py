import cv2
import cv2.aruco as aruco
import numpy as np

def detect_aruco_pose(frame, aruco_dict, parameters, camera_matrix, dist_coeffs):
    """
    Detect the ArUco tag and estimate its pose (translation and rotation vectors).
    
    Args:
        frame: The current video frame from the webcam.
        aruco_dict: The ArUco dictionary used to detect the tag.
        parameters: ArUco detection parameters.
        camera_matrix: The camera matrix from calibration.
        dist_coeffs: Distortion coefficients from calibration.

    Returns:
        tvec: Translation vector (position in 3D space).
        rvec: Rotation vector (orientation in 3D space).
        frame: The frame with detected markers and axis drawn.
    """
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the ArUco markers in the grayscale frame
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected
    if ids is not None:
        # Estimate the pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

        # Draw the detected markers and their axes on the frame
        for i in range(len(ids)):
            # Draw axis for the detected marker
            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            # Draw the detected marker boundary
            aruco.drawDetectedMarkers(frame, corners)

        # Return the first detected marker's pose
        return tvecs[0], rvecs[0], frame  # Translation vector, Rotation vector
    else:
        return None, None, frame  # No markers detected
