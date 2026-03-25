import time
import math
import cv2
import serial
import ydlidar
import pytesseract
import numpy as np

def send_command(arduino, command):
    arduino.write(command.encode())
    print(f"[SERIAL] Sent: {command}")
######################################################
#  camera                                                                                         CAMERA
######################################################
def check_victim(side):
    if side == "right":
        x = 0
    else: 
        x = 1 
    cap = cv2.VideoCapture(x)
    config = r'--oem 3 --psm 10 -c tessedit_char_whitelist=HSU'
    detected_text = None
    while True:
        ret, frame = cap.read()
        if not ret:
            print("no frame")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
        white_pixels = np.sum(th > 127)
        total_pixels = th.shape[0] * th.shape[1]
        text_ratio = white_pixels / total_pixels
        if text_ratio > 0.01: 
            text = pytesseract.image_to_string(th, config=config).strip()           
            if text:
                print("Detected:", text)
                detected_text = text
                break  
    cap.release()
    cv2.destroyAllWindows()
    return detected_text
####################################################################
# initialize lidar                                                                             initialize lidar
####################################################################
def get_rays(scan, tolerance=1):
    target_rays = {
        "front_left_ray":  -85,
        "front": -90,
        "front_right_ray": -95,
        "right_top_ray":   -3,
        "right": 0,
        "right_bot_ray":    3,
        "back_left_ray":    85,
        "back": 90,
        "back_right_ray":   95,
        "left_neg_ray":   -178,
        "left": 180,
        "left_pos_ray":    178,
        }
    rays = {name: None for name in target_rays}
    best = {name: float('inf') for name in target_rays}
    for point in scan.points:
        angle_deg = math.degrees(point.angle)
        dist = point.range * 100
        for name, target in target_rays.items():
            diff = abs(angle_deg - target)
            if diff < tolerance and diff < best[name]:
                best[name] = diff
                rays[name] = dist
    for name in rays:
        if rays[name] is None:
            rays[name] = 0
    return rays

if __name__ == "__main__":
    try:
        arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        time.sleep(2)
        print("Arduino connected on /dev/ttyUSB1")
    except serial.SerialException as e:
        raise RuntimeError(f"Failed to connect to Arduino: {e}")
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for key, value in ports.items():
        port = value
        print(port)
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
    ret = laser.initialize()
    if not ret:
        arduino.close()
        raise RuntimeError("Failed to initialize LiDAR")
    ret = laser.turnOn()
    if not ret:
        laser.disconnecting()
        arduino.close()
        raise RuntimeError("Failed to turn on LiDAR")
    scan = ydlidar.LaserScan()
    try:
        while ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            if r:
                rays = get_rays(scan)
                print(f"Front: {rays['front']:.2f}, Right: {rays['right']:.2f}, "
                      f"Back: {rays['back']:.2f}, Left: {rays['left']:.2f}")
    finally:
        laser.turnOff()
        laser.disconnecting()
######################################################################################
# moving commands                                                                                     moving commands 
######################################################################################
def check_directions(scan, threshold_cm=30):
    distances = get_rays(scan)
    front = distances.get(-90, 0)
    right = distances.get(0, 0)
    back  = distances.get(90, 0)
    left  = distances.get(180, 0)
    return {
        "front": 0 < front <= threshold_cm,
        "right": 0 < right <= threshold_cm,
        "back":  0 < back  <= threshold_cm,
        "left":  0 < left  <= threshold_cm,
    }
def move_one_tile(laser, scan, arduino, distance_to_move=30):
    distances = get_rays(scan)
    front_distance = distances[-90]
    if front_distance == 0:
        print("No valid front distance detected. Aborting move.")
        return
    target_distance = front_distance - distance_to_move
    send_command(arduino, 'F60')
    while front_distance > target_distance:
        r = laser.doProcessSimple(scan)
        if r:
            distances = get_rays(scan)
            front_distance = distances[-90]
            if front_distance == 0:
                print("Lost front reading during movement. Stopping.")
                send_command(arduino, 'S')
                return
        else:
            print("Failed to get LiDAR data during movement, retrying...")
        time.sleep(0.05)
    send_command(arduino, 'S')
def turn(laser, scan, arduino, direction, turn_time=0.6, tolerance_cm=1.5):
    command = 'L60' if direction == "left" else 'R60'
    send_command(arduino, command)
    time.sleep(turn_time)
    send_command(arduino, 'S')
    time.sleep(0.3)

    while True:
        r = laser.doProcessSimple(scan)
        if not r:
            time.sleep(0.05)
            continue
        
        rays = get_rays(scan)

        if direction == "right":
            ray_a = rays["right_top_ray"]   
            ray_b = rays["right_bot_ray"]  
        else:
            ray_a = rays["left_pos_ray"]   
            ray_b = rays["left_neg_ray"] 
        if ray_a == 0 or ray_b == 0:
            time.sleep(0.05)
            continue
        diff = abs(ray_a - ray_b)
        print(f"Fixing: {ray_a:.2f} : {ray_b:.2f} diff: {diff:.2f}")
        if diff <= tolerance_cm:
            send_command(arduino, 'S')
            print(f"Aligned. ray_a: {ray_a:.2f} ray_b: {ray_b:.2f}")
            return
        if ray_a > ray_b:
            send_command(arduino, 'R60')
        else:
            send_command(arduino, 'L60')
        time.sleep(0.1)
##############################################################################
#                 DFS                                                                             DFS DFS DFS DFS DFS DFS DFS
##############################################################################
# class Node:
#     def __init__(self, state, parent):
#         self.state = state
#         self.parent = parent

# class RobotDFS:
#     def __init__(self, laser, scan, arduino, grid_size=5):
#         self.laser = laser
#         self.scan = scan
#         self.arduino = arduino
#         self.grid_size = grid_size
#         self.visited = set()
#         self.current_pos = (0, 0)
#         self.direction = 0  
#         self.victims = []
    
#     def solve(self):
#         stack = [Node(self.current_pos, None)]
        
#         while stack:
#             node = stack.pop()
            
#             if node.state in self.visited:
#                 continue
            
#             self.visited.add(node.state)
#             self.current_pos = node.state
#             print(f"Exploring: {node.state}")
            
#             # Check for victim at current position
#             victim = check_victim("right")
#             if victim:
#                 print(f"Victim found at {node.state}: {victim}")
#                 self.victims.append((node.state, victim))
            
#             # Get neighbors
#             neighbors = self.get_valid_neighbors(node.state)
            
#             for next_state in neighbors:
#                 if next_state not in self.visited:
#                     stack.append(Node(next_state, node))
#                     # Move robot to next state
#                     self.move_to(next_state)
        
#         print(f"Exploration complete. Victims found: {self.victims}")
#         return self.victims
    
#     def get_valid_neighbors(self, state):
#         i, j = state
#         neighbors = []
        
#         # Check all 4 directions
#         directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
#         for di, dj in directions:
#             ni, nj = i + di, j + dj
#             # Check if within grid
#             if 0 <= ni < self.grid_size and 0 <= nj < self.grid_size:
#                 # Check if no obstacle
#                 r = self.laser.doProcessSimple(self.scan)
#                 if r:
#                     directions_check = check_directions(self.scan)
#                     # Map grid direction to robot direction
#                     if di == -1 and directions_check["front"]:  # Moving up
#                         neighbors.append((ni, nj))
#                     elif di == 1 and directions_check["back"]:   # Moving down
#                         neighbors.append((ni, nj))
#                     elif dj == -1 and directions_check["left"]:  # Moving left
#                         neighbors.append((ni, nj))
#                     elif dj == 1 and directions_check["right"]:  # Moving right
#                         neighbors.append((ni, nj))
        
#         return neighbors
    
#     def move_to(self, next_state):
#         current_i, current_j = self.current_pos
#         next_i, next_j = next_state
        
#         # Calculate direction to move
#         if next_i < current_i:  # Move forward
#             move_one_tile(self.laser, self.scan, self.arduino)
#         elif next_i > current_i:  # Move backward
#             turn(self.laser, self.scan, self.arduino, "left")
#             turn(self.laser, self.scan, self.arduino, "left")
#             move_one_tile(self.laser, self.scan, self.arduino)
#         elif next_j < current_j:  # Move left
#             turn(self.laser, self.scan, self.arduino, "left")
#             move_one_tile(self.laser, self.scan, self.arduino)
#         elif next_j > current_j:  # Move right
#             turn(self.laser, self.scan, self.arduino, "right")
#             move_one_tile(self.laser, self.scan, self.arduino)