import time
import math
import cv2
import serial
import ydlidar
import pytesseract
import numpy as np

def send_command(arduino, command):
    arduino.write((command + '\n').encode())
    print(f"[SERIAL] Sent: {command}")
######################################################
#  camera                                                                                         CAMERA
######################################################
def check_victim(side):
    if side == "right":
        x = 0
        flip = True
    else:
        x = 1
        flip = False
    
    cap = cv2.VideoCapture(x)
    config = r'--oem 3 --psm 6 -c tessedit_char_whitelist=HSU'
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if flip:
            frame = cv2.flip(frame, 0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)
        
        ratio = np.sum(th > 127) / (th.shape[0] * th.shape[1])

        if ratio > 0.005:
            try:
                text = pytesseract.image_to_string(th, config=config, timeout=5).strip().upper()
                text = text[0] 

                if text in ["H", "S", "U"]:
                    print(f"[VICTIM] {text}")
                    send_command(arduino , text)
            except Exception as e:
                print(f"[VICTIM] OCR error: {e}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cap.release()
    cv2.destroyAllWindows()

#########################################################################################################
# LIDAR AND SENSING
#########################################################################################################

def get_rays(scan, tolerance=1):
    target_rays = {
        "front": -90,
        "right": 0,
        "left": 180,
        "right_top_ray": -3,
        "right_bot_ray": 3,
        "left_pos_ray": 178,
        "left_neg_ray": -178,
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

def read_neighbors(laser, scan):
    r = laser.doProcessSimple(scan)
    if not r:
        return []
    
    rays = get_rays(scan)
    front = rays["front"]
    left = rays["left"]
    right = rays["right"]
    
    print(f"[SENSE] Front: {front:.1f}cm, Left: {left:.1f}cm, Right: {right:.1f}cm")
    
    neighbors = []
    if front > 20:
        neighbors.append("F")
    if left > 20:
        neighbors.append("L")
    if right > 20:
        neighbors.append("R")
    
    print(f"[NEIGHBORS] Available: {neighbors}")
    return neighbors

#########################################################################################################
# MOVEMENT
#########################################################################################################

def move_one_tile(laser, scan, arduino, distance_to_move=30):
    r = laser.doProcessSimple(scan)
    if not r:
        print("[MOVE] Failed to get initial LiDAR reading")
        return False

    distances = get_rays(scan)
    if not distances:
        print("[MOVE] Invalid ray data")
        return False
    
    front_distance = distances["front"]
    right_distance = distances["right"]
    left_distance = distances["left"]

    if front_distance == 0 or front_distance < distance_to_move:
        print("[MOVE] Front blocked")
        return False

    target_distance = front_distance - distance_to_move
    initial_offset = right_distance - left_distance

    base_speed = 80
    correction_step = 20

    send_command(arduino, f'F{base_speed},F{base_speed}')

    while True:
        r = laser.doProcessSimple(scan)
        if not r:
            send_command(arduino, 'S')
            return False

        distances = get_rays(scan)
        if not distances:
            send_command(arduino, 'S')
            return False
        
        front_distance = distances["front"]
        right_distance = distances["right"]
        left_distance = distances["left"]

        current_offset = right_distance - left_distance
        error = current_offset - initial_offset

        left_speed = base_speed
        right_speed = base_speed

        if abs(error) > 1.5:
            if error > 0:
                right_speed = max(base_speed - correction_step, 0)
            else:
                left_speed = max(base_speed - correction_step, 0)
        
        send_command(arduino, f'F{right_speed},F{left_speed}')
        time.sleep(0.1)

        if front_distance <= target_distance:
            send_command(arduino, 'S')
            print(f"[MOVE] Reached tile. Front distance: {front_distance:.2f}cm")
            return True

def turn(laser, scan, arduino, direction, turn_time=0.6, tolerance_cm=0.5):
    command = 'L80' if direction == "left" else 'R80'
    send_command(arduino, command)
    time.sleep(turn_time)
    send_command(arduino, 'S')
    time.sleep(0.3)

    print(f"[TURN] Aligning {direction}...")
    
    while True:
        r = laser.doProcessSimple(scan)
        if not r:
            time.sleep(0.05)
            continue
        
        rays = get_rays(scan)
        if not rays:
            time.sleep(0.05)
            continue

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
        
        if diff <= tolerance_cm:
            send_command(arduino, 'S')
            print(f"[TURN] Aligned. ray_a: {ray_a:.2f}cm, ray_b: {ray_b:.2f}cm")
            return True
        
        if ray_a > ray_b:
            send_command(arduino, 'R60')
        else:
            send_command(arduino, 'L60')
        time.sleep(0.1)
#######################################################
def rwf(laser, scan, arduino):
    while True:
        r = laser.doProcessSimple(scan)
        if not r:
            time.sleep(0.05)
            continue
        
        rays = get_rays(scan)
        if not rays:
            time.sleep(0.05)
            continue
        
        front = rays["front"]
        right = rays["right"]
        left = rays["left"]
        
        print(f"Check: right={right:.1f}, front={front:.1f}")
        
        if right > 33:
            print("No right wall - turning right")
            send_command(arduino, "S")
            time.sleep(1)
            turn(laser, scan, arduino, "right")
        elif front > 0 and front <= 20:
            print("Right wall + front obstacle - turning left")
            send_command(arduino, "S")
            time.sleep(1)
            turn(laser, scan, arduino, "left")
        else:
            print("Right wall + clear front - going forward")
            move_one_tile(laser, scan, arduino, distance_to_move=30)
            time.sleep(0.2)


#########################################################################################################
# DFS MAZE SOLVER
#########################################################################################################

class Tile():
    def __init__(self, state, parent, action):
        self.state = state  # (x, y) position
        self.parent = parent
        self.action = action  # "F", "L", "R"

class StackFrontier():
    def __init__(self):
        self.frontier = []
    
    def add(self, tile):
        self.frontier.append(tile)
    
    def empty(self):
        return len(self.frontier) == 0
    
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        tile = self.frontier[-1]
        self.frontier = self.frontier[:-1]
        return tile

def get_neighbors(state, available_directions):
    x, y = state
    neighbors = []
    
    direction_map = {
        "F": (x, y + 1, "F"),
        "L": (x - 1, y, "L"),
        "R": (x + 1, y, "R"),
    }
    
    for direction in available_directions:
        if direction in direction_map:
            next_x, next_y, action = direction_map[direction]
            neighbors.append((action, (next_x, next_y)))
    
    return neighbors

def dfs_solve(laser, scan, arduino, start, max_steps=100):
    start_tile = Tile(start, None, None)
    frontier = StackFrontier()
    frontier.add(start_tile)
    explored = set()
    steps = 0
    path_to_return = None  
    
    print("[DFS] === EXPLORATION PHASE ===")
    
    while not frontier.empty() and steps < max_steps:
        steps += 1
        current_tile = frontier.remove()
        current_state = current_tile.state
        
        print(f"\n[DFS] Step {steps}: at {current_state}")
        
        if current_state in explored:
            continue
        
        explored.add(current_state)
        
        available = read_neighbors(laser, scan)
        neighbors = get_neighbors(current_state, available)
        
        print(f"[DFS] Available neighbors: {neighbors}")
        
        found_unvisited = False
        for action, next_state in neighbors:
            if next_state not in explored:
                found_unvisited = True
                print(f"[DFS] Moving {action} to {next_state}")
                
                if action == "F":
                    if not move_one_tile(laser, scan, arduino):
                        print("[DFS] Forward move failed")
                        continue
                elif action == "L":
                    if not turn(laser, scan, arduino, "left"):
                        print("[DFS] Left turn failed")
                        continue
                elif action == "R":
                    if not turn(laser, scan, arduino, "right"):
                        print("[DFS] Right turn failed")
                        continue
                
                new_tile = Tile(next_state, current_tile, action)
                frontier.add(new_tile)
                
                time.sleep(0.5)
                break  
        
        if not found_unvisited:
            path_to_return = current_tile
            print(f"[DFS] Dead end at {current_state}. Mapping return path...")
        
        time.sleep(0.3)
    
    print("\n[DFS] === EXPLORATION COMPLETE ===")
    print(f"[DFS] Explored {len(explored)} tiles")
    
    # RETURN PHASE
    # ============
    
    print("\n[DFS] === RETURN PHASE ===")
    print("[DFS] Returning to start...")
    
    if path_to_return is None:
        path_to_return = current_tile
    
    return_path = []
    current = path_to_return
    while current.parent is not None:
        return_path.append((current.action, current.state))
        current = current.parent
    
    return_path.reverse()
    
    print(f"[DFS] Return path: {return_path}")
    
    for i, (action, position) in enumerate(return_path):
        print(f"\n[RETURN] Step {i+1}/{len(return_path)}: Going back via {action} to {position}")
        
        if action == "F":
            if not move_one_tile(laser, scan, arduino):
                print("[RETURN] Forward move failed")
        elif action == "L":
            if not turn(laser, scan, arduino, "right"):
                print("[RETURN] Right turn failed")
        elif action == "R":
            if not turn(laser, scan, arduino, "left"):
                print("[RETURN] Left turn failed")
        
        time.sleep(0.5)
    
    print("\n[DFS] === BACK AT START ===")
    return True

#########################################################################################################
# INITIALIZATION
#########################################################################################################

def init_arduino():
    try:
        arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        time.sleep(2)
        print("[INIT] Arduino connected on /dev/ttyUSB1")
        return arduino
    except serial.SerialException as e:
        raise RuntimeError(f"Failed to connect to Arduino: {e}")

def init_lidar():
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for key, value in ports.items():
        port = value
        print(f"[INIT] LiDAR port: {port}")
    
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
        raise RuntimeError("Failed to initialize LiDAR")
    
    ret = laser.turnOn()
    if not ret:
        laser.disconnecting()
        raise RuntimeError("Failed to turn on LiDAR")
    
    print("[INIT] LiDAR initialized")
    return laser

#########################################################################################################
# MAIN
#########################################################################################################

if __name__ == "__main__":
    arduino = None
    laser = None
    scan = None
    
    try:
        print("[START] Initializing robot...")
        arduino = init_arduino()
        laser = init_lidar()
        scan = ydlidar.LaserScan()
        rwf(laser,scan,arduino)
        # print("[START] Starting maze solve...")
        # start_pos = (0, 0)
        # dfs_solve(laser, scan, arduino, start_pos, max_steps=100)
        
    except Exception as e:
        print(f"[ERROR] {e}")
    
    finally:
        print("[SHUTDOWN] Stopping robot...")
        if arduino:
            send_command(arduino, 'S')
        if laser:
            laser.turnOff()
            laser.disconnecting()
        if arduino:
            arduino.close()
        print("[SHUTDOWN] Done")