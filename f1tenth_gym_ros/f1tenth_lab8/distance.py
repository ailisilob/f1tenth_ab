import cv2
import numpy as np
import glob

def calibrate_camera(calibration_folder='calibration'):
    CHECKERBOARD = (6, 8)
    square_size = 25.0  # 25mm per square

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0],
                            0:CHECKERBOARD[1]].T.reshape(-1, 2) * square_size

    objpoints = []
    imgpoints = []

    images = sorted(glob.glob(f'{calibration_folder}/*.png') +
                    glob.glob(f'{calibration_folder}/*.jpg'))
    print(f"Found {len(images)} calibration images")

    for fname in images:
        img  = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(f"  Corners found: {fname}")
        else:
            print(f"  Corners NOT found: {fname}")

    print(f"\nUsing {len(objpoints)} images for calibration")
    h, w = cv2.imread(images[0]).shape[:2]
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None)

    print(f"\nIntrinsic matrix K:\n{K}")
    print(f"Distortion coefficients:\n{dist}")
    print(f"Reprojection error: {ret:.4f} pixels")
    return K, dist

def compute_height(K, u, v, known_x_car_mm=400.0):
    # H = known_x_car * fx / (u - cx)
    fx = K[0, 0]
    cx = K[0, 2]
    H = known_x_car_mm * fx / (u - cx)
    print(f"\nCamera mounting height H = {H:.1f} mm")
    return H

def pixel_to_car(u, v, K, H):
    # Convert image pixel (u=col, v=row) to car frame (x=forward, y=left)
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    x_car = (u - cx) / fx * H
    y_car = (v - cy) / fy * H
    return x_car, y_car

if __name__ == '__main__':
    # Step 1: Camera calibration
    K, dist = calibrate_camera('calibration')

    # Step 2: Compute camera height using cone_x40cm.png
    # Pixel coordinate of lower-right corner of nearest red cone (manually measured)
    u_40cm, v_40cm = 664, 493
    H = compute_height(K, u_40cm, v_40cm, known_x_car_mm=400.0)

    # Step 3: Measure unknown cone distance using cone_unknown.png
    # Pixel coordinate of lower-right corner of cone (manually measured)
    u_unk, v_unk = 597, 415
    x_car, y_car = pixel_to_car(u_unk, v_unk, K, H)
    print(f"\nUnknown cone distance:")
    print(f"  x_car = {x_car:.1f} mm  ({x_car/10:.1f} cm)")
    print(f"  y_car = {y_car:.1f} mm  ({y_car/10:.1f} cm)")