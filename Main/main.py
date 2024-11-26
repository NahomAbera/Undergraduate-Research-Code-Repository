#necessary libraries
import sys
import rospy  
from std_msgs.msg import String
import cv2
import numpy as np
import serial
import csv
import math
import time
import os
from datetime import datetime

#robotic arm control path
sys.path.append('/home/ubuntu/software/armpi_pro_control')

#robotic arm controller
from BusServoCmd import Controller
controller = Controller()

#ROS node
rospy.init_node("autonomous_navigation", anonymous=True)

velocity_publisher = rospy.Publisher('/robot_velocity', String, queue_size=10)

# Initialize serial communication with Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

#initialize camera for obstacle detection
cap = cv2.VideoCapture(0)

#YOLO model for obstacle detection
net = cv2.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

with open('coco.names', 'r') as f:
    classes = f.read().splitlines()

#global variables for current position and heading
curr_x = 0.0
curr_y = 0.0
curr_heading = 0.0  #radians

#movement control functions
def set_velocity(speed, direction, rotation_speed):
    """
    Sends velocity commands to the robot.
    :param speed: Linear speed (-100 to 100 mm/s)
    :param direction: Movement direction in degrees (0-360)
    :param rotation_speed: Angular speed (-2 to 2 rad/s)
    """
    command = f"{speed},{direction},{rotation_speed}"
    velocity_publisher.publish(command)

def move_forward(speed=60):
    set_velocity(speed, 90, 0)

def turn_left(speed=60):
    set_velocity(speed, 180, 0)

def turn_right(speed=60):
    set_velocity(speed, 0, 0)

def stop():
    set_velocity(0, 0, 0)

# Position tracking functions
def get_current_position():
    '''Implement position tracking using odometry or sensors
    Return the current x and y coordinates'''
    global curr_x, curr_y
    # .....
    return curr_x, curr_y

def get_current_heading():
    '''Implement heading tracking using gyroscope or IMU
    Return the current heading in radians'''
    global curr_heading
    # ....
    return curr_heading

def update_position():
    '''Update curr_x, curr_y, curr_heading based on sensor data
    This function should be called regularly to keep track of the robot's position'''
    pass  

def at_position(target_x, target_y, tolerance=0.5):
    current_x, current_y = get_current_position()
    distance = math.hypot(target_x - current_x, target_y - current_y)
    return distance < tolerance

#Eexecute robotic arm actions
def execute_action(action_name, wait_time=2):
    """
    Executes a predefined action file using the robotic arm controller.
    """
    controller.runAction(action_name)
    time.sleep(wait_time)

# Obstacle detection function
def obstacle_detected():
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera.")
        return False

    # Preprocess the image for YOLO
    height, width, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    output_layers_names = net.getUnconnectedOutLayersNames()
    layerOutputs = net.forward(output_layers_names)

    # Initialize lists
    boxes = []
    confidences = []
    class_ids = []

    # Process detections
    for output in layerOutputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # Confidence threshold
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                # Bounding box coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                # Append to lists
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply Non-Max Suppression
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    obstacle_present = False
    for i in indexes.flatten():
        label = str(classes[class_ids[i]])
        obstacle_present = True
        break

    return obstacle_present

def avoid_obstacle():
    stop()
    turn_right(speed=60)
    time.sleep(1)
    move_forward(speed=60)
    time.sleep(1)
    stop()
    update_position()

# Serial communication with Arduino for sensor data
def request_sensor_data():
    ser.write(b'REQUEST\n')
    line = ser.readline().decode('utf-8').strip()
    if line:
        parts = line.split('|')
        if len(parts) == 3:
            moisture_value = parts[0].split(':')[1].strip()
            moisture_percent = parts[1].split(':')[1].strip().replace('%', '')
            temperature_c = parts[2].split(':')[1].strip().replace('°C', '')
            return moisture_value, moisture_percent, temperature_c
    return None, None, None

def collect_data_at_point(x, y):
    # Ensure the robot is stopped
    stop()

    try:
        # Execute arm action to lower sensor
        execute_action('lower_sensor', wait_time=2)

        # Allow time for the sensor to stabilize
        time.sleep(1)

        # Request sensor data from Arduino
        moisture_value, moisture_percent, temperature_c = request_sensor_data()

        # Execute arm action to raise sensor
        execute_action('raise_sensor', wait_time=2)

    except Exception as e:
        print(f"Error during data collection at ({x}, {y}): {e}")
        moisture_value, moisture_percent, temperature_c = None, None, None

    # Return data along with coordinates
    return {
        'x': x,
        'y': y,
        'moisture_value': moisture_value,
        'moisture_percent': moisture_percent,
        'temperature_c': temperature_c
    }

def log_data(quadrant, data_point):
    filename = f'quadrant_{quadrant}_data.csv'
    file_exists = os.path.isfile(filename)
    with open(filename, 'a', newline='') as csvfile:
        fieldnames = ['x', 'y', 'moisture_value', 'moisture_percent', 'temperature_c', 'timestamp']
        csvwriter = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if not file_exists:
            csvwriter.writeheader()  # Write header if file doesn't exist

        csvwriter.writerow({
            'x': data_point['x'],
            'y': data_point['y'],
            'moisture_value': data_point['moisture_value'],
            'moisture_percent': data_point['moisture_percent'],
            'temperature_c': data_point['temperature_c'],
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })

#Function to generate waypoints for each quadrant
def generate_quadrant_waypoints(quadrant, center_x, center_y, max_x, max_y, min_x, min_y, coverage_step):
    waypoints = []
    if quadrant == 1:
        #Quadrant 1: center to (max_x, max_y)
        x_start, x_end = center_x, max_x
        y_start, y_end = center_y, max_y
    elif quadrant == 2:
        #Quadrant 2: center to (min_x, max_y)
        x_start, x_end = center_x, min_x
        y_start, y_end = center_y, max_y
    elif quadrant == 3:
        #Quadrant 3: center to (min_x, min_y)
        x_start, x_end = center_x, min_x
        y_start, y_end = center_y, min_y
    elif quadrant == 4:
        #Quadrant 4: center to (max_x, min_y)
        x_start, x_end = center_x, max_x
        y_start, y_end = center_y, min_y
    else:
        return waypoints

    #generate waypoints in a grid pattern
    x_positions = np.arange(x_start, x_end, coverage_step)
    y_positions = np.arange(y_start, y_end, coverage_step)

    if quadrant in [1, 4]:
        x_positions = x_positions
    else:
        x_positions = x_positions[::-1]

    if quadrant in [1, 2]:
        y_positions = y_positions
    else:
        y_positions = y_positions[::-1]


    for i, y in enumerate(y_positions):
        if i % 2 == 0:
            for x in x_positions:
                waypoints.append((x, y))
        else:
            for x in reversed(x_positions):
                waypoints.append((x, y))
    waypoints.append((center_x, center_y))

    return waypoints

def move_to(target_x, target_y):
    tolerance = 0.5  
    max_speed = 60  
    angular_speed = 1.0  

    while not at_position(target_x, target_y, tolerance):
        update_position()
        curr_x, curr_y = get_current_position()
        curr_heading = get_current_heading()

        delta_x = target_x - curr_x
        delta_y = target_y - curr_y
        distance = math.hypot(delta_x, delta_y)
        angle_to_target = math.atan2(delta_y, delta_x)

        heading_error = angle_to_target - curr_heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        if abs(heading_error) > 0.1:  
            rotation_direction = 1 if heading_error > 0 else -1
            if rotation_direction > 0:
                turn_left(speed=int(angular_speed * 30))  
            else:
                turn_right(speed=int(angular_speed * 30))
            time.sleep(abs(heading_error) / angular_speed)
            stop()
        else:
            #move forward towards the target
            speed = min(max_speed, int(distance * 1000))  #convert distance to mm/s
            move_forward(speed)
            time.sleep(distance / (speed / 1000))  #Convert speed back to m/s for time calculation
            stop()

        # Check for obstacles
        if obstacle_detected():
            avoid_obstacle()
            # After avoidance, recalculate path to target
            continue

    stop()

def main():
    global curr_x, curr_y, curr_heading

    #current position at the center
    field_length = 100.0  #Adjustable based on feld
    field_width = 100.0 #Adjustable based on field
    curr_x = field_length / 2
    curr_y = field_width / 2
    curr_heading = 0.0 

    #field dimensions and center
    center_x = field_length / 2
    center_y = field_width / 2
    coverage_step = 5.0  #Adjustable based on needed data point spacing

    min_x = 0.0
    max_x = field_length
    min_y = 0.0
    max_y = field_width

    #initialize arm to starting position
    execute_action('arm_start_position', wait_time=2)

    rate = rospy.Rate(10)  # 10 Hz loop rate

    #loop through each quadrant
    for quadrant in [1, 2, 3, 4]:
        if rospy.is_shutdown():
            break
        waypoints = generate_quadrant_waypoints(quadrant, center_x, center_y, max_x, max_y, min_x, min_y, coverage_step)

        #navigate to each waypoint in the quadrant
        for waypoint in waypoints:
            if rospy.is_shutdown():
                break

            target_x, target_y = waypoint
            print(f"Navigating to ({target_x}, {target_y})")
            move_to(target_x, target_y)

            #collect data at the point
            data_point = collect_data_at_point(target_x, target_y)
            #log data to the appropriate CSV file
            log_data(quadrant, data_point)

            rate.sleep()

    #Move arm to rest position
    execute_action('arm_rest_position', wait_time=2)

    cap.release()
    ser.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        cap.release()
        ser.close()