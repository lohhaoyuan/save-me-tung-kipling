import time
import math
import struct

import ulab
import image
import sensor
from pyb import UART, LED
from ulab import numpy as np


# --------------------------------------------- CONFIG ---------------------------------------------

DEBUG_CAMERA = True

# Camera system
# ----- OLDBOT -----
CAMERA_CENTER = np.array((159, 140))
CAMERA_ROI = (32, 14, 244, 227)
HMIRROR = True
VFLIP = False
TRANSPOSE = False
"""
0 6
10 10
20 17
30 24
40 40
50 51
60 58
70 65
80 68
90 72
100 76
110 79
120 83
130 84
140 90
150 92
160 95
170 99
180 100
190 101
200 102
"""
DISTANCE_COEFFICIENTS = np.array(
    [-3.49578147e-06, 1.03941994e-03, -8.13682248e-02, 3.22518576e00, -1.57496452e01],
    # [ 9.53340581e-05, -1.11504149e-02,  9.92479690e-01,  1.75040766e+00]
    # [2.45641451e-06, -4.13791515e-04, 2.54199085e-02, 1.71702709e-01, -7.67467924e-02]
)
# ----- NEWBOT -----
# CAMERA_CENTER = np.array((146, 157))
# CAMERA_ROI = (0, 0, 240, 320)
## CAMERA_CENTER = np.array((125, 177))
## CAMERA_ROI = (19, 76, 210, 210)
# HMIRROR = False
# VFLIP = False
# TRANSPOSE = True
# """
# 0 12
# 10 13
# 20 50
# 30 66
# 40 75
# 50 86
# 60 98
# 70 103
# 80 109
# 90 113
# 100 117
# 110 120
# 120 124
# 130 124
# 140 126
# 150 128
# 160 131
# 170 131
# 180 132
# 190 133
# 200 133
# """
# DISTANCE_COEFFICIENTS = np.array(
# [ 1.17041523e-09, -4.21577792e-07,  5.77787509e-05, -3.65929209e-03,
# 1.05876533e-01, -6.34293210e-01, -2.28222842e+00]
# )
# ------------------

# Camera configuration
GAIN = 20
# RGB_GAIN = (-6, -20, -20)
RGB_GAIN = (-7.0, -7.0, -1.0)
# print(sensor.get_gain_db())
# print(sensor.get_exposure_us() * 0.21)

# Blob tracking
# ----- Computer Lab 3 -----
# BLUE_THRESHOLDS = [(35, 50, -10, 30, -70, -40)]
# YELLOW_THRESHOLDS = [(50, 75, -30, 0, 40, 80)]
# --------- AI Lab ---------
# BLUE_THRESHOLDS = [(45, 65, -30, 5, -40, -10)]
# YELLOW_THRESHOLDS = [(45, 80, 0, 40, 40, 80)]
# ---------- Home ----------
# BLUE_THRESHOLDS = [(25, 60, -15, 25, -60, -20)]
# YELLOW_THRESHOLDS = [(50, 90, -20, 5, 40, 80)]
# ---------- 318 ----------
## oldbot
# BLUE_THRESHOLDS = [(18, 33, -128, 15, -128, -3)]
# YELLOW_THRESHOLDS = [(64, 100, -128, 127, 59, 127)]
# EXPOSURE = 20000
## newbot
# BLUE_THRESHOLDS = [(0, 100, -128, 127, -128, -17)]
# YELLOW_THRESHOLDS = [(58, 100, -44, 12, 52, 127)]
# EXPOSURE = 20000
# ---------- lw only practice field 1640 ----------
# BLUE_THRESHOLDS = [(0, 60, -128, 127, -128, -10)]
# YELLOW_THRESHOLDS = [(35, 100, -16, 127, 30, 127)]
# GAIN = 20
# EXPOSURE = 13000
# ---------- lw only practice field 1130/1230 ----------
# BLUE_THRESHOLDS = [(20, 35, -128, 127, -128, -8)]
# YELLOW_THRESHOLDS = [(41, 100, -16, 127, 13, 127)]
# GAIN = 20
# EXPOSURE = 5000
# ---------- match 1 1000 ----------
## oldbot
## newbot
# BLUE_THRESHOLDS = [(0, 60, -36, -5, -128, -10)]
# YELLOW_THRESHOLDS = [(63, 100, -13, 127, 33, 127)]
##GAIN = 20
# EXPOSURE = 13000
# ---------- match 4 ----------
## oldbot
## newbot
BLUE_THRESHOLDS = [(20, 54, -128, -3, -128, -11)]
YELLOW_THRESHOLDS = [(61, 100, -15, 127, 17, 127)]
# GAIN = 20
EXPOSURE = 1775
# ---------- sus field -------
# BLUE_THRESHOLDS = [(29, 53, -128, 6, -128, -16)]
# YELLOW_THRESHOLDS = [(50, 100, -21, 127, 33, 127)]
##GAIN = 20
# EXPOSURE = 9000
#
BLOB_MERGE = False
BLOB_STRIDE = 1
BLOB_PIXEL_THRESHOLD = 15
BLOB_AREA_THRESHOLD = 15
BLOB_MARGIN = 5

F = np.eye(6, dtype=np.float)
B = 0
H = np.array(
    [
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
    ],
    dtype=np.float,
)
# Q represents process noise
# larger values reduce response time, smaller values reduce noise
Q = np.array(
    [
        [1e-3, 0, 0, 0, 0, 0],
        [0, 1e-2, 0, 0, 0, 0],
        [0, 0, 1e-2, 0, 0, 0],
        [0, 0, 0, 1e-2, 0, 0],
        [0, 0, 0, 0, 1e-2, 0],
        [0, 0, 0, 0, 0, 1e-3],
    ],
    dtype=np.float,
)
# R represents measurement noise
# smaller values indicate greater measurement precision
R = np.array(
    [
        [1e-3, 0, 0, 0],
        [0, 1e-3, 0, 0],
        [0, 0, 1e-3, 0],
        [0, 0, 0, 1e-3],
    ],
    dtype=np.float,
)

# --------------------------------------------- SETUP ----------------------------------------------

LED(2).off()  # Turn off the green LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(HMIRROR)
sensor.set_vflip(VFLIP)
sensor.set_transpose(TRANSPOSE)
sensor.skip_frames(time=1000)

sensor.set_auto_gain(False, gain_db=17)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE)
sensor.set_auto_whitebal(
    False, rgb_gain_db=RGB_GAIN
)  # Must be turned off for blob tracking
sensor.set_brightness(0)
sensor.set_contrast(3)
sensor.set_saturation(3)
sensor.skip_frames(time=1000)

# while True:
# print(sensor.get_exposure_us())
# img = sensor.snapshot()

serial = UART(3, 1000000)

# -------------------------------------------- METHODS ---------------------------------------------


def pixels_to_cm(x):
    return np.polyval(DISTANCE_COEFFICIENTS, [x])[0]


def find_object(img, lab_threshold, draw_color):
    # Find blobs
    blobs = img.find_blobs(
        lab_threshold,
        merge=BLOB_MERGE,
        roi=CAMERA_ROI,
        x_stride=BLOB_STRIDE,
        y_stride=BLOB_STRIDE,
        pixels_threshold=BLOB_PIXEL_THRESHOLD,
        area_threshold=BLOB_AREA_THRESHOLD,
        margin=BLOB_MARGIN,
    )

    # Find the largest blob
    max_area = 0
    detected_blob = None
    for blob in blobs:
        if blob.area() > max_area:
            max_area = blob.area()
            detected_blob = blob

    if not detected_blob:
        return None

    # Find the base of the blob
    mal = detected_blob.minor_axis_line()
    base1 = np.array(mal[0:2])
    base2 = np.array(mal[2:4])
    base1 -= CAMERA_CENTER
    base2 -= CAMERA_CENTER
    base1_distance = math.sqrt(base1[0] ** 2 + base1[1] ** 2)
    base2_distance = math.sqrt(base2[0] ** 2 + base2[1] ** 2)
    position = base1 if base1_distance < base2_distance else base2
    distance = base1_distance if base1_distance < base2_distance else base2_distance

    # Annotate the blob if we're debugging
    if DEBUG_CAMERA:
        img.draw_ellipse(detected_blob.enclosed_ellipse(), color=draw_color)
        absolute_position = CAMERA_CENTER + position
        img.draw_cross(
            int(absolute_position[0]), int(absolute_position[1]), color=draw_color
        )

    # Find the relative polar coordinates of the object
    angle = (math.degrees(math.atan2(position[1], position[0])) + 90) % 360
    distance = pixels_to_cm(distance)

    return angle, distance


def cobs_encode(input_bytes):
    read_index = 0
    write_index = 1
    code_index = 0
    code = 1

    output_bytes = bytearray(
        len(input_bytes) + 2
    )  # have enough space for the output data

    while read_index < len(input_bytes):
        if input_bytes[read_index] == 0:
            output_bytes[code_index] = code
            code = 1
            code_index = write_index
            write_index += 1
            read_index += 1
        else:
            output_bytes[write_index] = input_bytes[read_index]
            write_index += 1
            read_index += 1
            code += 1
            if code == 0xFF:
                output_bytes[code_index] = code
                code = 1
                code_index = write_index
                write_index += 1

    output_bytes[code_index] = code

    return output_bytes[:write_index]


# class KalmanFilter:
# def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None) -> None:
# if F is None or H is None:
# raise ValueError("Set proper system dynamics.")

## Initialise parameters
# self.n = F.shape[1]
# self.m = H.shape[1]
# self.F = F
# self.H = H
## self.B = np.zeros(1, dtype=np.float) if B is None else B
# self.Q = np.eye(self.n, dtype=np.float) if Q is None else Q
# self.R = np.eye(self.n, dtype=np.float) if R is None else R
# self.P = np.zeros((self.n, self.n)) if P is None else P
# self.x = None

# self._last_time = None

# def predict(self) -> np.ndarray:
## Compute dt at each prediction step
# if self._last_time is None:
# self._last_time = time.time()
# return None
# current_time = time.time()
# dt = current_time - self._last_time
# self._last_time = current_time
# self.F[0][2] = dt
# self.F[1][3] = dt

## Predict
# self.x = np.dot(self.F, self.x)  # + np_dot( self.B, u )
# self.P = np.dot(np.dot(self.F, self.P), self.F.transpose().copy()) + self.Q
# return self.x

# def update(self, z) -> None:
## Initialise parameters if it's the first update
# if self.P is None:
# self.P = np.eye(self.F.shape[1])
# if self.x is None:
# self.x = np.array([z[0], z[1], [0], [0], z[2], z[3]], dtype=np.float)

## Update state
# y = z - np.dot(self.H, self.x)
# S = self.R + np.dot(self.H, np.dot(self.P, self.H.transpose().copy()))
# K = np.dot(np.dot(self.P, self.H.transpose().copy()), np.linalg.inv(S))
# self.x = self.x + np.dot(K, y)
# I = np.eye(self.n)
# self.P = np.dot(
# np.dot(I - np.dot(K, self.H), self.P),
# (I - np.dot(K, self.H)).transpose().copy(),
# ) + np.dot(np.dot(K, self.R), K.transpose().copy())


# ---------------------------------------------- LOOP ----------------------------------------------


# blue_goal_filter = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)
# yellow_goal_filter = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)
blue_goal_not_found_count = 0
yellow_goal_not_found_count = 0
endurance = 100
while True:
    try:
        img = sensor.snapshot()
    except RuntimeError:
        continue

    blue_goal = find_object(img, BLUE_THRESHOLDS, (0, 0, 255))
    yellow_goal = find_object(img, YELLOW_THRESHOLDS, (0, 255, 0))
    # if blue_goal:
    # blue_goal_filter.update(np.array([[blue_goal[0]], [blue_goal[1]], [0], [0]], dtype=np.float))
    # else:
    # blue_goal_not_found_count += 1
    # if yellow_goal:
    # yellow_goal_filter.update(np.array([[yellow_goal[0]], [yellow_goal[1]], [0], [0]], dtype=np.float))
    # else:
    # yellow_goal_not_found_count += 1

    # blue_goal_state = blue_goal_filter.predict()
    # blue_goal = (blue_goal_state[0][0], blue_goal_state[1][0]) if blue_goal_state and blue_goal_not_found_count <= endurance else None
    # yellow_goal_state = yellow_goal_filter.predict()
    # yellow_goal = (yellow_goal_state[0][0], yellow_goal_state[1][0]) if yellow_goal_state else None

    # Pack data
    buf = struct.pack(
        "<dddd",
        blue_goal[0] if blue_goal else float("nan"),  # d, double
        blue_goal[1] if blue_goal else float("nan"),  # d, double
        yellow_goal[0] if yellow_goal else float("nan"),  # d, double
        yellow_goal[1] if yellow_goal else float("nan"),  # d, double
    )

    # Encode with COBS
    buf = cobs_encode(buf)
    buf += b"\x00"  # delimiter byte

    # Send packet
    serial.write(buf)

    if DEBUG_CAMERA:
        img.draw_cross(
            int(CAMERA_CENTER[0]), int(CAMERA_CENTER[1]), color=(255, 255, 255)
        )
        img.draw_rectangle(CAMERA_ROI, color=(255, 255, 255))

        if blue_goal:
            print(f"Blue Goal: {blue_goal[0]:.1f}º {blue_goal[1]:.1f} cm", end=" | ")
        else:
            print("Blue Goal:                ", end=" | ")
        if yellow_goal:
            print(f"Yellow Goal: {yellow_goal[0]:.1f}º {yellow_goal[1]:.1f} cm")
        else:
            print("Yellow Goal:                ")

# --------------------------------------------------------------------------------------------------
