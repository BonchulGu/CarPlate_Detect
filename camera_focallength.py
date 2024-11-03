# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import serial
import time
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("best.pt")

# Constants
ACTUAL_PLATE_HEIGHT = 0.11  # meters (actual height of the license plate)
FOCAL_LENGTH = 550  # initial focal length in pixels

# Serial connection to Arduino
arduino_port = "COM5"  # Replace with your COM port
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)

# Variable to store previous ultrasonic distance data
previous_sensor_distance = None  # Store previous ultrasonic distance

# Calculate distance from camera to license plate
def calculate_distance(plate_height_pixels):
    return (ACTUAL_PLATE_HEIGHT * FOCAL_LENGTH) / plate_height_pixels

# Calculate estimated focal length based on ultrasonic distance
def calculate_focal_length(actual_distance, plate_height_pixels):
    return (actual_distance * plate_height_pixels) / ACTUAL_PLATE_HEIGHT

# Order corners to ensure consistent ordering
def order_corners(corners):
    center = np.mean(corners, axis=0)
    ordered = sorted(corners, key=lambda p: (-math.atan2(p[1] - center[1], p[0] - center[0]), p[1], p[0]))
    if ordered[1][1] < ordered[0][1]:
        ordered[0], ordered[1] = ordered[1], ordered[0]
    if ordered[3][1] < ordered[2][1]:
        ordered[2], ordered[3] = ordered[3], ordered[2]
    return np.array(ordered)

# Calculate plate height in pixels
def get_plate_height(corners):
    ordered_corners = order_corners(corners)
    top_midpoint = ((ordered_corners[0][0] + ordered_corners[1][0]) // 2, (ordered_corners[0][1] + ordered_corners[1][1]) // 2)
    bottom_midpoint = ((ordered_corners[2][0] + ordered_corners[3][0]) // 2, (ordered_corners[2][1] + ordered_corners[3][1]) // 2)
    plate_height_pixels = math.sqrt((top_midpoint[0] - bottom_midpoint[0]) ** 2 + (top_midpoint[1] - bottom_midpoint[1]) ** 2)
    return plate_height_pixels, top_midpoint, bottom_midpoint

# Draw information on frame
def draw_info(frame, distance, corners, mid_top, mid_bottom, sensor_distance, focal_length_estimate):
    # Draw the bounding box and distance if license plate is detected
    if distance is not None:
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
        cv2.putText(frame, f"Distance: {distance:.2f} m", (mid_top[0], mid_top[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.circle(frame, mid_top, 5, (0, 0, 255), -1)
        cv2.circle(frame, mid_bottom, 5, (0, 0, 255), -1)
        cv2.line(frame, mid_top, mid_bottom, (255, 0, 0), 2)

    # Always display ultrasonic sensor distance on bottom-left
    if sensor_distance is not None:
        cv2.putText(frame, f"Ultrasonic Distance: {sensor_distance:.2f} cm", 
                    (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Display estimated focal length on top-left if calculated
    if focal_length_estimate is not None:
        cv2.putText(frame, f"Estimated Focal Length: {focal_length_estimate:.2f} px", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO prediction
    results = model.predict(source=frame, show=False)

    plate_height_pixels = None
    distance = None
    sensor_distance = previous_sensor_distance  # Use previous value if no new data is available
    focal_length_estimate = None

    # Process each detected object
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

                    # Calculate the plate height in pixels and the midpoints
                    plate_height_pixels, mid_top, mid_bottom = get_plate_height(corners)

                    # Calculate the distance using the fixed focal length
                    distance = calculate_distance(plate_height_pixels)

                    # Calculate estimated focal length using ultrasonic distance
                    if sensor_distance is not None:
                        focal_length_estimate = calculate_focal_length(sensor_distance / 100, plate_height_pixels)

                    # Draw the bounding box and distance information
                    draw_info(frame, distance, corners, mid_top, mid_bottom, sensor_distance, focal_length_estimate)

    # Read ultrasonic sensor data and update if new data is available
    if serial_conn.in_waiting > 0:
        try:
            sensor_distance = float(serial_conn.readline().decode('utf-8').strip())
            previous_sensor_distance = sensor_distance  # Update previous distance
        except ValueError:
            pass  # Ignore any conversion errors

    # Draw ultrasonic distance and focal length even if no plate is detected
    draw_info(frame, None, None, None, None, sensor_distance, focal_length_estimate)

    # Show frame
    cv2.imshow("License Plate Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
serial_conn.close()
