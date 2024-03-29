import cv2
import mediapipe as mp
import time
import math
import rclpy
from rclpy.node import Node
from bistable_interfaces.msg import TrackingData
from time import sleep

# Initialize Mediapipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# ROS 2 Node for publishing target positions
class TargetPositionsPublisher(Node):
    def __init__(self):
        super().__init__('target_positions_publisher')
        self.publisher_ = self.create_publisher(TrackingData, '/tracking_data', 10)

    def publish_position(self, inc, yaw_vel):
        msg = TrackingData()
        msg.inclination = inc
        msg.yaw_vel = yaw_vel

        self.publisher_.publish(msg)

def map_val(value, from_low, from_high, to_low, to_high):
    # Ensure the value is within the original range
    value = max(from_low, min(value, from_high))
    # Calculate the mapped value
    mapped_value = to_low + (value - from_low) * (to_high - to_low) / (from_high - from_low)
    return mapped_value

def inversely_map_val(value, from_low, from_high, to_low, to_high):
    # Ensure the value is within the original range
    value = max(from_low, min(value, from_high))
    # Calculate the mapped value inversely
    mapped_value = to_low + (to_high - to_low) - (value - from_low) * (to_high - to_low) / (from_high - from_low)
    return mapped_value

def is_palm(hand_coordinates, recognition_threshold, hand_side):
    percentage_error = 0
    if hand_side == "right":
        target_angles = [-2.525, -1.948, -1.61, -1.7, 0.434, -1.613, -1.575, -1.53, 1.365, -1.622, -1.597, -1.597, 1.391, -1.595, -1.594, -1.618, 1.386, -1.561, -1.625, -1.664]
    elif hand_side == "left":
        target_angles = [-0.647, -1.234, -1.568, -1.339, 2.576, -1.533, -1.602, -1.622, 1.766, -1.526, -1.553, -1.538, 1.754, -1.534, -1.49, -1.466, 1.781, -1.573, -1.466, -1.426]

    angles = [] 
    for i in range(21):
        if i != 20:
            x = hand_coordinates[i][0]
            y = hand_coordinates[i][1]
            x_next = hand_coordinates[i+1][0]
            y_next = hand_coordinates[i+1][1]
            angle = math.atan2(y_next - y, x_next - x)
            angles.append(angle)

    for j in range(21):
        if j != 20:
            percentage_error += abs((target_angles[j] - angles[j]) / target_angles[j]) * 100

    if percentage_error / 20 <= recognition_threshold:
        return True
    else:
        return False

def find_relative_distance(hand_coordinates):
    relative_distance = 0
    # Calculate relative distance between hand joints
    for i in range(4, 20, 4):
        x, y, _ = hand_coordinates[i]
        x_next, y_next, _ = hand_coordinates[i + 4]
        relative_distance += math.sqrt((x_next - x) ** 2 + (y_next - y) ** 2)

    return relative_distance

def find_actual_distance(relative_distance):
    m = 0.143053
    c = -0.013
    return (1 / relative_distance) * m + c

def main(args=None):

    rclpy.init(args=args)
    node = TargetPositionsPublisher()

    # Video capture using OpenCV
    cap = cv2.VideoCapture()
    cap.open(0, apiPreference=cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30.0)

    recognition_threshold = 15
    target_distance = 0.8
    interval = 0.002

    kp_distance = 0.5
    ki_distance = 0.15
    kd_distance = 0.0
    integral_distance = 0
    prev_distance_error = 0

    kp_yaw = 0.5
    ki_yaw = 0.1
    kd_yaw = 0.0
    integral_yaw = 0
    prev_yaw_error = 0

    while True:
        # sleep(interval)
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Extract the coordinates of all hand joints
                hand_coordinates = [(landmark.x, landmark.y, landmark.z) for landmark in hand_landmarks.landmark]
                if is_palm(hand_coordinates, recognition_threshold, hand_side="right"):
                    
                    relative_distance = find_relative_distance(hand_coordinates)
                    actual_distance = find_actual_distance(relative_distance)

                    # PID for distance
                    distance_error = target_distance - actual_distance
                    integral_distance += distance_error * interval
                    derivative_distance = (distance_error - prev_distance_error) / interval
                    drive_command = (kp_distance * distance_error) + (ki_distance * integral_distance) + (kd_distance * derivative_distance)
                    prev_distance_error = distance_error

                    # PID for steering
                    hand_x = hand_coordinates[9][0]
                    yaw_error = 0.5 - hand_x
                    integral_yaw += yaw_error * interval
                    derivative_yaw = (yaw_error - prev_yaw_error) / interval
                    steer_command = (kp_yaw * yaw_error) + (ki_yaw * integral_yaw) + (kd_yaw * derivative_yaw)
                    prev_yaw_error = yaw_error
                

                    # Publish the mapped value to ROS 2 topic
                    # sleep(interval)
                    node.publish_position(drive_command, steer_command)

                else:
                    node.publish_position(0.0, 0.0)

        else:
            node.publish_position(0.0, 0.0)

        # Display the frame
        cv2.imshow("Hand Tracking", frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close all windows
    cap.release()
    cv2

