import numpy as np
import cv2
import glob
import argparse

parser = argparse.ArgumentParser(prog='overhead-analyser', description='Overhead Field Analyser')

parser.add_argument('-d', '--video-device', nargs='?', help='Video device number, eg. for /dev/video2 supply \"2\". You might need to be root.', type=int, default=0)
parser.add_argument('-s', '--size', nargs='?', help='Width that the video device runs at, eg. \"1280,720\"', default="1280,720")
parser.add_argument('--field-width', nargs='?', help='Width of the long side of the field in the final unwrapped image', type=int, default=1000)
parser.add_argument('--field-height', nargs='?', help='Width of the short side of the field in the final unwrapped image', type=int, default=666)
parser.add_argument('-K', '--intrinsic-matrix', nargs='?', help='Calibrated fisheye camera intrinsic matrix. Use this, --calibrate.')
parser.add_argument('-D', '--distortion-coefficients', nargs='?', help='Fisheye distortion coefficients. Use this, or --calibrate.')
parser.add_argument('-c', '--calibrate', help='Calculate fisheye distortion coefficients (-D) and camera intrinsic matrix (-K) from images with checkerboards of same size as video device.', action='store_true')
parser.add_argument('-S', '--undistort-scale', nargs='?', help='How much to zoom out the fisheye-corrected image so the field fits in the frame', type=float, default=0.56)
parser.add_argument('-p', '--checker-path', nargs='?', help='Glob pattern for images of a checkerboard taken on this camera, eg. \"data/*.jpg\"', default='../../desktop_ws/resources/lab_camera_checkerboards/*.jpg')
parser.add_argument('--checker-width', nargs='?', help='Number of squares horizontally on the checkerboard', type=int, default=12)
parser.add_argument('--checker-height', nargs='?', help='Number of squares vertically on the checkerboard', type=int, default=8)
parser.add_argument('-N', '--no-unwrap', help='Do fisheye undistortion, but do not do the overhead unwrap. Arguments p1, p2, p3, p4 do nothing if this is on.', action='store_true')
parser.add_argument('-p1', '--top-left', nargs='?', help='Coordinates of the top-left corner of the field in the original fully distorted image, eg. x,y', type=float)
parser.add_argument('-p2', '--top-right', nargs='?', help='Coordinates of the top-right corner of the field in the original fully distorted image.', type=float)
parser.add_argument('-p3', '--bottom-left', nargs='?', help='Coordinates of the bottom-left corner of the field in the original fully distorted image.', type=float)
parser.add_argument('-p4', '--bottom-right', nargs='?', help='Coordinates of the bottom-right corner of the field in the original fully distorted image.', type=float)

args = parser.parse_args()

checkerboard = (args.checker_width, args.checker_height)
w, h = tuple(map(int, args.size.split(',')))

K = np.zeros((3, 3))
D = np.zeros((4, 1))

if args.calibrate:
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

    objp = np.zeros((1, checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2)

    objpoints = []
    imgpoints = []
    counter = 0
    for path in glob.glob(args.checker_path):
        # Load the image and convert it to gray scale
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # images must be the same size
        assert gray.shape[0] == h
        assert gray.shape[1] == w
        # h, w = gray.shape

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), subpix_criteria)
            imgpoints.append(corners)
            
            print(f'Found checkerboard in {path}, using to compute undistortion...')
        else:
            print(f'No checkerboard found in {path}, skipping...')


    print(f'Calculating fisheye distortion using {len(objpoints)} images...')

    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]

    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        (w, h),
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

    print(f'Camera intrinsic matrix before scale of {args.undistort_scale}x (pass with -K next time):', repr(K).replace('\n', '').replace(' ', '').lstrip('array(').rstrip(')'))
    print('Fisheye distortion coefficients (pass with -D next time):', repr(D).replace('\n', '').replace(' ', '').lstrip('array(').rstrip(')'))
    # print()
else:
    if args.intrinsic_matrix is None or args.distortion_coefficients is None:
        print('You have not supplied camera intrinsic matrix / distortion coefficients. \nhint: If you don\'t know and want to solve for these from checkerboard images, re-run with \"-c\".')
        exit(1)
    
    # yes using eval is cooked but you gotta do what you gotta do
    K = np.array(eval(args.intrinsic_matrix), dtype=np.float32)
    D = np.array(eval(args.distortion_coefficients), dtype=np.float32)

# print(f'Scaling K by {args.undistort_scale}x!')
Knew_scaled = K.copy()
Knew_scaled[:2, :2] *= args.undistort_scale

overhead_field_width = args.field_width
overhead_field_height = args.field_height

TOP_LEFT     = ((w - overhead_field_width) / 2, (h - overhead_field_height) / 2)
TOP_RIGHT    = (TOP_LEFT[0] + overhead_field_width, TOP_LEFT[1])
BOTTOM_LEFT  = (TOP_LEFT[0], TOP_LEFT[1] + overhead_field_height)
BOTTOM_RIGHT = (TOP_LEFT[0] + overhead_field_width, TOP_LEFT[1] + overhead_field_height)

p1 = (297, 101)
p2 = (943, 96)
p3 = (61, 525)
p4 = (1192, 481)

if args.top_left:
    p1 = tuple(map(float, args.top_left.split(',')))

if args.top_right:
    p2 = tuple(map(float, args.top_right.split(',')))

if args.bottom_left:
    p3 = tuple(map(float, args.bottom_left.split(',')))

if args.bottom_right:
    p4 = tuple(map(float, args.bottom_right.split(',')))

# Points in the distorted image
corners_of_field_in_fisheye = np.array([[p1], [p2], [p3], [p4]], dtype=np.float32)

# Corresponding points in the ideal rectangle (known size)
corners_of_field_ideal = np.array([TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT], dtype=np.float32)

def onClick(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        print(x, y)


# perform fisheye undistortion of 
corners_of_field_undistorted = cv2.fisheye.undistortPoints(corners_of_field_in_fisheye, K=K, D=D, P=Knew_scaled)

# Compute homography
H, _ = cv2.findHomography(corners_of_field_undistorted, corners_of_field_ideal)

cap = cv2.VideoCapture(args.video_device)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

while True:
    ret, img = cap.read()
    if not ret:
        print('Failed to get camera frame! Is the video device valid, and do you have permissions to read it?')
        break

    # perform fisheye undistortion of image
    undistorted_img = cv2.fisheye.undistortImage(img, K=K, D=D, Knew=Knew_scaled, new_size=(w, h))
    
    cv2.imshow('Original Image', img)
    cv2.imshow('Undistorted Image', undistorted_img)
    
    if not args.no_unwrap:
        # Apply perspective transform
        overhead_image = cv2.warpPerspective(undistorted_img, H, (w, h))
        cv2.imshow('Overhead Image', overhead_image)

    cv2.setMouseCallback("Original Image", onClick)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()