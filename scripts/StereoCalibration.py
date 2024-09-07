import os
import cv2
import glob
import numpy as np
import socket
import time

def find_image_files(base_path, prefix, count):
    image_files = []
    #read all file
    image_files = glob.glob(os.path.join(base_path, "*.png"))
    print(len(image_files))
    #sort by name
    image_files.sort()
    print(image_files)
    # for i in range(1, count + 1):
    #     filename = os.path.join(base_path, f"{prefix}_{i}.png")
    #     if os.path.exists(filename):
    #         image_files.append(filename)
    #     else:
    #         print(f"Missing image file: {filename}")
    #         break

    return image_files

def calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    return mean_error / len(objpoints)

def compute_stereo_reprojection_error(objpoints, imgpoints_left, imgpoints_right, mtx_left, dist_left, mtx_right, dist_right, R, T):
    mean_error_left = 0
    mean_error_right = 0
    total_points = 0

    for i in range(len(objpoints)):
        imgpoints2_left, _ = cv2.projectPoints(objpoints[i], np.zeros((3, 1)), np.zeros((3, 1)), mtx_left, dist_left)
        imgpoints2_right, _ = cv2.projectPoints(objpoints[i], R, T, mtx_right, dist_right)

        error_left = cv2.norm(imgpoints_left[i], imgpoints2_left, cv2.NORM_L2) / len(imgpoints2_left)
        error_right = cv2.norm(imgpoints_right[i], imgpoints2_right, cv2.NORM_L2) / len(imgpoints2_right)

        mean_error_left += error_left
        mean_error_right += error_right
        total_points += len(objpoints[i])

    mean_error_left /= len(objpoints)
    mean_error_right /= len(objpoints)
    return mean_error_left, mean_error_right

def single_camera_calibration(images, square_size, pattern_size):
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    objpoints = []
    imgpoints = []

    for img_path in images:
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners2)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    mean_error = calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist)
    print(f"Single Camera Reprojection Error: {mean_error}")
    return ret, mean_error, mtx, dist

def stereo_calibration(left_images, right_images, square_size, pattern_size, mtx_left, dist_left, mtx_right, dist_right):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    objpoints = []
    imgpoints_left = []
    imgpoints_right = []

    for img_left_path, img_right_path in zip(left_images, right_images):
        img_left = cv2.imread(img_left_path)
        img_right = cv2.imread(img_right_path)

        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        ret_left, corners_left = cv2.findChessboardCorners(gray_left, pattern_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, pattern_size, None)

        if ret_left and ret_right:
            corners2_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
            corners2_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints_left.append(corners2_left)
            imgpoints_right.append(corners2_right)

    ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right, mtx_left, dist_left, mtx_right, dist_right, gray_left.shape[::-1],
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)

    # Compute the reprojection errors for left and right cameras.
    mean_error_left, mean_error_right = compute_stereo_reprojection_error(objpoints, imgpoints_left, imgpoints_right, mtx_left, dist_left, mtx_right, dist_right, R, T)

    print(f"Left Camera mean reprojection error: {mean_error_left}")
    print(f"Right Camera mean reprojection error: {mean_error_right}")

    np.savez('stereo_calibration_data.npz',
        rms_error=ret,                                              # The root mean square (RMS) re-projection error.
        left_camera_matrix=mtx_left,                                # The camera matrix for the left camera.
        left_distortion_coefficients=dist_left,                     # The distortion coefficients for the left camera.
        right_camera_matrix=mtx_right,                              # The camera matrix for the right camera.
        right_distortion_coefficients=dist_right,                   # The distortion coefficients for the right camera.
        rotation_matrix=R,                                          # The rotation matrix between the two camera coordinate systems.
        translation_vector=T,                                       # The translation vector between the camera coordinate systems.
        essential_matrix=E,                                         # The essential matrix.
        fundamental_matrix=F)                                       # The fundamental matrix.
    
    print("Stereo calibration complete.")


def send_calibration_complete_signal():
    attempts = 5
    for _ in range(attempts):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(('127.0.0.1', 12346))
                s.sendall(b'Calibration Complete')
                break
        except ConnectionRefusedError:
            time.sleep(1)

def main():
    image_num = 108
    square_size = 0.006
    pattern_size = (10, 12)
    left_images = find_image_files("LEFT", "LEFT", image_num)
    right_images = find_image_files("RIGHT", "RIGHT", image_num)


    if len(left_images) == image_num and len(right_images) == image_num:
        print("Calibrating left camera...")
        _, _, mtx_left, dist_left = single_camera_calibration(left_images, square_size, pattern_size)

        print("Calibrating right camera...")
        _, _, mtx_right, dist_right = single_camera_calibration(right_images, square_size, pattern_size)

        print("Performing stereo calibration...")
        stereo_calibration(left_images, right_images, square_size, pattern_size, mtx_left, dist_left, mtx_right, dist_right)

        send_calibration_complete_signal()
    else:
        print("Not enough images for calibration. Found left: {}, right: {}.".format(len(left_images), len(right_images)))

    print("Stereo Camera Calibration")
    send_calibration_complete_signal()

if __name__ == "__main__":
    main()