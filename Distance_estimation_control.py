# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import serial
import time
from ultralytics import YOLO
import threading

# Load YOLO model
model = YOLO("best.pt")

# Constants for distance calculation
ACTUAL_PLATE_HEIGHT = 0.11  # meters
FOCAL_LENGTH = 550  # pixels

# Serial connection to Arduino
arduino_port = "COM5"
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)

# Timer and state variables
sensor_distance = None
is_moving = False  # Track if the vehicle is moving
is_reversing = False  # Track if the vehicle is reversing
last_detection_time = time.time()  # Initialize last detection time

# Initialize timers and flags for maintaining 0.5-second distance checks
start_time_1m = None
start_time_50cm = None
within_1m_for_0_5s = False
within_50cm_for_0_5s = False

# Calculate distance based on plate height in pixels
def calculate_distance(plate_height_pixels):
    return (ACTUAL_PLATE_HEIGHT * FOCAL_LENGTH) / plate_height_pixels

# Order corners in consistent manner
def order_corners(corners):
    center = np.mean(corners, axis=0)
    ordered = sorted(corners, key=lambda p: (-math.atan2(p[1] - center[1], p[0] - center[0]), p[1], p[0]))
    if ordered[1][1] < ordered[0][1]:
        ordered[0], ordered[1] = ordered[1], ordered[0]
    if ordered[3][1] < ordered[2][1]:
        ordered[2], ordered[3] = ordered[3], ordered[2]
    return np.array(ordered)

# Get plate height in pixels
def get_plate_height(corners):
    ordered_corners = order_corners(corners)
    mid_top = ((ordered_corners[0][0] + ordered_corners[1][0]) // 2, (ordered_corners[0][1] + ordered_corners[1][1]) // 2)
    mid_bottom = ((ordered_corners[2][0] + ordered_corners[3][0]) // 2, (ordered_corners[2][1] + ordered_corners[3][1]) // 2)
    plate_height_pixels = math.sqrt((mid_top[0] - mid_bottom[0]) ** 2 + (mid_top[1] - mid_bottom[1]) ** 2)
    return plate_height_pixels, mid_top, mid_bottom

# Detection function with automatic control based on distance
def yolo_detection():
    global start_time_1m, start_time_50cm, sensor_distance, is_moving, is_reversing, last_detection_time
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(source=frame, conf=0.5, verbose=False, show=False)
        plate_detected = False
        plate_distance = None
        current_time = time.time()

        for result in results:
            if result.masks is None:
                continue

            for j in range(result.masks.data.shape[0]):
                segmentation_mask = result.masks.data.cpu().numpy()[j]
                contours, _ = cv2.findContours(segmentation_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    contour = max(contours, key=cv2.contourArea)
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)

                    if len(approx) == 4:
                        corners = [tuple(point[0]) for point in approx]
                        plate_height_pixels, mid_top, mid_bottom = get_plate_height(corners)
                        plate_distance = calculate_distance(plate_height_pixels)
                        plate_detected = True  # Mark that plate is detected
                        last_detection_time = current_time  # Update last detection time

                        # Display plate distance on top of the plate
                        cv2.putText(frame, f"Plate Distance: {plate_distance:.2f} m", 
                                    (mid_top[0], mid_top[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                        # Draw bounding box
                        for i in range(4):
                            cv2.line(frame, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)

        # Check if 5 seconds have passed since the last plate detection
        if not plate_detected and (current_time - last_detection_time > 5):
            # Reset to manual mode if 5 seconds have passed without plate detection
            is_moving = False
            is_reversing = False

        # Automatic control with 0.5-second delay at 50cm and 1m during reverse
        if plate_distance is not None and is_reversing:
            # Check if plate_distance is within 50cm and maintain for 0.5 second
            if plate_distance <= 0.5:
                if start_time_50cm is None:
                    start_time_50cm = current_time
                elif current_time - start_time_50cm >= 0.5:
                    if not within_50cm_for_0_5s:
                        serial_conn.write(b'S')  # Stop motor
                        cv2.putText(frame, "Stopping at 50cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        print("Stopping at 50cm")
                        is_moving = False
                        is_reversing = False  # Ensure reversing is also set to false
                        within_50cm_for_0_5s = True  # Flag that 50cm condition is met
            else:
                start_time_50cm = None
                within_50cm_for_0_5s = False

            # Check if plate_distance is within 1m and maintain for 0.5 second
            if plate_distance <= 1.0:
                if start_time_1m is None:
                    start_time_1m = current_time
                elif current_time - start_time_1m >= 0.5:
                    if not within_1m_for_0_5s:
                        serial_conn.write(b'L')  # Slow down
                        print("Slowing down at 1m")
                        within_1m_for_0_5s = True  # Flag that 1m condition is met
            else:
                start_time_1m = None
                within_1m_for_0_5s = False

        # Display the frame
        cv2.imshow("License Plate Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting program...")
            break

    cap.release()
    cv2.destroyAllWindows()
    serial_conn.close()

# User input loop for manual control
def user_input():
    global is_moving, is_reversing
    print("Commands: 'w' = Forward, 's' = Backward, 'x' = Stop, 'q' = Quit")

    while True:
        command = input("Enter command: ").strip()
        
        if command == 'w':
            serial_conn.write(b'F')
            is_moving = True
            is_reversing = False  # Forward mode
            print("Sent 'F' for Forward")
        elif command == 's':
            serial_conn.write(b'B')
            is_moving = True
            is_reversing = True  # Reverse mode
            print("Sent 'B' for Backward")
        elif command == 'x':
            serial_conn.write(b'S')
            is_moving = False
            print("Sent 'S' for Stop")
        elif command == 'q':
            print("Exiting program...")
            break
        else:
            print("Invalid command. Please enter 'w', 's', 'x', or 'q'.")

# Start detection in a thread
yolo_thread = threading.Thread(target=yolo_detection)
input_thread = threading.Thread(target=user_input)

yolo_thread.start()
input_thread.start()

yolo_thread.join()
input_thread.join()
