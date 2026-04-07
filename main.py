import time
import math
import operator
import serial
import ydlidar
import numpy as np
import cv2
from collections import deque
from collections import namedtuple
from enum import IntEnum


def nearest_neighbor(src, dst):
    indices = []
    for p in src:
        dists = np.linalg.norm(dst - p, axis=1)
        idx = np.argmin(dists)
        indices.append(idx)
    return indices
 
def best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
 
    AA = A - centroid_A
    BB = B - centroid_B
 
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
 
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
 
    t = centroid_B - R @ centroid_A
    return R, t
 
def icp(A, B, iterations=10):
    src = np.array(A)
    dst = np.array(B)
    
    for i in range(iterations):
        indices = nearest_neighbor(src, dst)
        dst_matched = dst[indices]
        R, t = best_fit_transform(src, dst_matched)
        src = src @ R.T + t
    
    R, t = best_fit_transform(A, np.array(B))
    return R, t

def scan_to_xy(scan, min_range=0.02, max_range=10.0):
    points = []
    for p in scan.points:
        if min_range < p.range < max_range:
            x = p.range * math.cos(p.angle)
            y = p.range * math.sin(p.angle)
            points.append([x, y])
    return np.array(points)

class RayBuffer:
    def __init__(self, size=5):
        self.buffers = {
            "front": deque(maxlen=size),
            "right": deque(maxlen=size),
            "left": deque(maxlen=size),
            "back": deque(maxlen=size),
            "right_top_ray": deque(maxlen=size),
            "right_bot_ray": deque(maxlen=size),
            "left_pos_ray": deque(maxlen=size),
            "left_neg_ray": deque(maxlen=size),
        }
    
    def add(self, rays):
        for name in self.buffers:
            if rays[name] != 0:
                self.buffers[name].append(rays[name])
    
    def get_median(self):
        result = {}
        for name, buf in self.buffers.items():
            if len(buf) == 0:
                result[name] = 0
            else:
                sorted_buf = sorted(buf)
                result[name] = sorted_buf[len(sorted_buf) // 2]
        return result
    
    def clear(self):
        for buf in self.buffers.values():
            buf.clear()

ray_buffer = RayBuffer(size=5)

def send_command(arduino, command):
    arduino.write((command + '\n').encode())
    print(f"[SERIAL] Sent: {command}")

def get_rays(scan, tolerance=1):
    target_rays = {
        "front": -90,
        "right": 0,
        "left": 180,
        "back": 90,
        "angle_-10": -10,
        "right_top_ray": -3,
        "right_bot_ray": 3,
        "angle_10": 10,
        "angle_170": 170,
        "left_pos_ray": 178,
        "left_neg_ray": -178,
        "angle_-170": -170
    }
    rays = {name: None for name in target_rays}
    best = {name: float('inf') for name in target_rays}
    
    for point in scan.points:
        angle_deg = math.degrees(point.angle)
        dist = point.range * 100

        if dist == 0:
            continue
        
        for name, target in target_rays.items():
            diff = abs(angle_deg - target)
            if diff < tolerance and diff < best[name]:
                best[name] = diff
                rays[name] = dist

    for name in rays:
        if rays[name] is None:
            rays[name] = 0  
    
    return rays

def flush_lidar_buffer(laser, scan, flushes=5):
    for _ in range(flushes):
        laser.doProcessSimple(scan)
        time.sleep(0.02)

def get_fresh_scan(laser, scan): 
    flush_lidar_buffer(laser, scan, flushes=3)
    r = laser.doProcessSimple(scan)
    if not r:
        return None
    return scan

def move_one_tile(laser, scan, arduino, distance_to_move=28, timeout_sec=5):
    ray_buffer.clear()
    time.sleep(0.2)
    if get_fresh_scan(laser, scan) is None:
        print("[MOVE] Failed to get initial LiDAR reading")
        return False
 
    distances = get_rays(scan)
    if not distances:
        print(" Invalid ray data")
        return False
    ray_buffer.add(distances)
    rays = ray_buffer.get_median()
    front_distance = rays["front"]
    right_distance = rays["right"]
    left_distance = rays["left"]

    if front_distance == 0 or front_distance < distance_to_move:
        print(" Front blocked")
        return False
 
    target_distance = max(front_distance - distance_to_move, 5)
    initial_right = ((right_distance + 15) % 30) - 15
    initial_left  = ((left_distance  + 15) % 30) - 15
    initial_offset = initial_right - initial_left
 
    base_speed = 60
    start_time = time.time()

    send_command(arduino, f'F{base_speed},F{base_speed}')
    
    while True:
        ray_buffer.clear()
        time.sleep(0.1)
        elapsed = time.time() - start_time
        if elapsed > timeout_sec:
            print(f" Timeout after {timeout_sec}s")
            send_command(arduino, 'S')
            return False
 
        if get_fresh_scan(laser, scan) is None:
            print("[MOVE] Failed to get initial LiDAR reading")
            return False
        distances = get_rays(scan)
        if not distances:
            print("[TURN] Invalid ray data")
            return False
        ray_buffer.add(distances)
        rays = ray_buffer.get_median()
        
        front_raw = rays["front"]
        right_distance = rays["right"]
        left_distance = rays["left"]

        left_speed = base_speed
        right_speed = base_speed

        modded_right_distance = ((right_distance + 15) % 30) - 15
        print(f'modded_right : {modded_right_distance}')
        modded_left_distance  = ((left_distance  + 15) % 30) - 15
        print(f'modded left : {modded_left_distance}')
        error = (modded_right_distance - modded_left_distance) - initial_offset
        print(f'intial {initial_offset}')
        print(f'error :{abs(error)}')
        g = operator.add
        j = operator.add  
        if abs(error) < 1:
            error = 0
            print("no change")
 
        elif abs(error) < 4:
            print("aligning left")
            g = operator.sub
            j = operator.add
        else:
            print("aliging right")
            g = operator.add
            j = operator.sub
            
        k = 0.9

        right_speed = g(base_speed , (k * error))
        left_speed  = j(base_speed  , (k * error))

        right_speed = max(min(int(right_speed), 60), 0)
        left_speed  = max(min(int(left_speed), 60), 0)

        send_command(arduino, f'F{right_speed},F{left_speed}')
        print("moving")

        if front_raw <= 10:
            send_command(arduino, 'S')
            print("no space ")
            time.sleep(0.2)
            ray_buffer.clear()
            time.sleep(0.5)
            return True
        if front_raw <= target_distance:
            send_command(arduino, 'S')
            print(f" Reached tile: {front_raw:.2f}cm")
            time.sleep(0.2)
            ray_buffer.clear()
            time.sleep(0.5)
            # if target_distance - 3 > front_raw:
            #     print("overshooted")
            #     send_command(arduino , 'B60,B60')
            #     time.sleep(0.07) 


            #align(laser,scan,arduino)
            return True
        
def better_turn(laser, scan, arduino, direction):
    ray_buffer.clear()
    if get_fresh_scan(laser, scan) is None:
        print("[MOVE] Failed to get initial LiDAR reading")
        return False
    
    distances = get_rays(scan)
    if not distances:
        print(" Invalid ray data")
        return False

    front = distances["front"]
    right = distances["right"]
    left =  distances["left"]
    modded_front = front
    modded_left = left
    modded_right = right

    if direction == "right":
        modded_front = right
        modded_left = front 
        condition = "FL"
        turn = "R40"
    else: 
        modded_front = left
        modded_right = front
        condition = "FR"      
        turn = "L40"
        
    send_command(arduino, turn)
    
    max_iter = 60
    for iteration in range(max_iter):
        if get_fresh_scan(laser, scan) is None:
            print("[MOVE] Failed to get initial LiDAR reading")
            return False
        distances = get_rays(scan)
        if not distances:
            print("[TURN] Invalid ray data")
            return False
        ray_buffer.add(distances)
        filtered = ray_buffer.get_median()

        front = filtered["front"]
        right = filtered["right"]
        left =  filtered["left"]

        err_front = abs(front - modded_front)
        err_right = abs(right - modded_right)
        err_left = abs(left - modded_left)

        tol_front = max(3, modded_front * 0.25)
        tol_right = max(15, modded_right * 0.15)
        tol_left  = max(15, modded_left * 0.15)

        aligend_front = err_front <= tol_front
        aligend_right = err_right <= tol_right
        aligned_left  = err_left  <= tol_left

        if condition == 'FR':
            print(f"F:{front:.0f}/{modded_front:.0f}(tol:{tol_front:.0f})={'✓' if aligend_front else '✗'} "
                f"R:{right:.0f}/{modded_right:.0f}(tol:{tol_right:.0f})={'✓' if aligend_right else '✗'}")
        else : 
            print(f"F:{front:.0f}/{modded_front:.0f}(tol:{tol_front:.0f})={'✓' if aligend_front else '✗'} "
                f"L:{left:.0f}/{modded_left:.0f}(tol:{tol_left:.0f})={'✓' if aligned_left else '✗'}")
            
        if aligend_front and aligend_right and condition == "FR":
            send_command(arduino, 'S')
            print("wasal belsama")
            ray_buffer.clear()
            time.sleep(0.5)
            if get_fresh_scan(laser, scan) is None:
                print("[MOVE] Failed to get initial LiDAR reading")
                return False
            distances = get_rays(scan)
            if not distances:
                print("[TURN] Invalid ray data")
                return False
            ray_buffer.add(distances)
            filtered = ray_buffer.get_median()

            right = filtered["right"]
            left =  filtered["left"]
            if right > left : 
                ray_a = filtered["left_pos_ray"]
                ray_b = filtered["left_neg_ray"]
            else:
                ray_a = filtered["right_top_ray"]
                ray_b = filtered["right_bot_ray"]
        
            diff = abs(ray_a - ray_b)
        
            if diff <= 1.5:
                send_command(arduino, 'S')
                print(f"[TURN] Aligned. ray_a: {ray_a:.2f}cm, ray_b: {ray_b:.2f}cm")
                ray_buffer.clear()
                time.sleep(1)
                return True
        
            correction_speed = int(20 + (diff * 3))
            correction_speed = min(80, correction_speed)
            
            if ray_a > ray_b:
                send_command(arduino, f'R{correction_speed}')
            else:
                send_command(arduino, f'L{correction_speed}')
            time.sleep(0.1)

            return True
        elif aligend_front and aligned_left and condition == "FL":
            send_command(arduino,'S')
            print("wasal belsalama")
            ray_buffer.clear()
            time.sleep(1)
            return True

        send_command(arduino, turn)
        time.sleep(0.05)
        print("stilllllllllllllllll")

def scan_to_xy(scan, min_range=0.05, max_range=10.0):
    """Convert a LaserScan to a numpy array of (x, y) points in meters."""
    points = []
    for p in scan.points:
        if min_range < p.range < max_range:
            x = p.range * math.cos(p.angle)
            y = p.range * math.sin(p.angle)
            points.append([x, y])
    return np.array(points)

def turn_90(laser, scan, arduino, direction, target_angle=90, tolerance=1.5):
    if direction not in ["left", "right"]:
        print("[TURN] Invalid direction")
        return False
    
    turn_cmd = "R40" if direction == "right" else "L40"
    ray_buffer.clear()
    time.sleep(0.2)
    
    # Get initial scan for ICP
    if get_fresh_scan(laser, scan) is None:
        print("[TURN] Failed to get initial LiDAR reading")
        return False
    
    prev_scan_points = scan_to_xy(scan)
    if len(prev_scan_points) < 5:
        print("[TURN] Insufficient scan points")
        return False
    
    total_theta = 0
    send_command(arduino, turn_cmd)
    
    # Phase 1: Rotation tracking via ICP
    max_rotation_iters = 100
    rotation_iter = 0
    
    while total_theta < target_angle and rotation_iter < max_rotation_iters:
        time.sleep(0.05)
        rotation_iter += 1
        
        if get_fresh_scan(laser, scan) is None:
            continue
        
        curr_scan_points = scan_to_xy(scan)
        if len(curr_scan_points) < 5:
            continue
        
        # Apply ICP to find rotation between scans
        try:
            R, t = best_fit_transform(prev_scan_points, curr_scan_points)
            delta_theta = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            
            # Only accumulate reasonable rotations
            if abs(delta_theta) < 15 and abs(delta_theta) > 0.1:
                total_theta += abs(delta_theta)
            
            prev_scan_points = curr_scan_points
            
            # Slow down when approaching target
            if total_theta > target_angle * 0.75:
                slow_cmd = "L30" if direction == "left" else "R30"
                send_command(arduino, slow_cmd)
                
        except Exception as e:
            print(f"[ICP] Error: {e}")
            continue
    
    send_command(arduino, 'S')
    time.sleep(0.1)
    print(f"[TURN] ICP rotation phase: {total_theta:.2f}° in {rotation_iter} iterations")
    
    # Phase 2: Fine-tuning using wall alignment
    ray_buffer.clear()
    max_fine_tune = 30
    fine_tune_iter = 0
    last_diff = float('inf')
    no_change_count = 0
    
    while fine_tune_iter < max_fine_tune:
        if get_fresh_scan(laser, scan) is None:
            time.sleep(0.05)
            fine_tune_iter += 1
            continue
        
        rays = get_rays(scan)
        ray_buffer.add(rays)
        filtered_rays = ray_buffer.get_median()
        
        # Get appropriate rays based on turn direction
        if direction == "left":
            # After left turn, front and right should be aligned
            front = filtered_rays.get("front", 0)
            right = filtered_rays.get("right", 0)
            
            if front == 0 or right == 0:
                fine_tune_iter += 1
                time.sleep(0.05)
                continue
            
            diff = abs(front - right)
            print(f"[FINE-TUNE] Front: {front:.1f}cm, Right: {right:.1f}cm, Diff: {diff:.1f}cm")
            
            if diff <= tolerance:
                send_command(arduino, 'S')
                print(f"[TURN] Complete. ICP: {total_theta:.2f}°, Fine-tune: {fine_tune_iter} steps")
                ray_buffer.clear()
                time.sleep(0.5)
                return True
            
            # Apply correction
            if front > right:
                # Turn right slightly
                correction_speed = min(30, int(15 + diff * 2))
                send_command(arduino, f'R{correction_speed}')
                time.sleep(0.08)
                send_command(arduino, 'S')
            else:
                # Turn left slightly
                correction_speed = min(30, int(15 + diff * 2))
                send_command(arduino, f'L{correction_speed}')
                time.sleep(0.08)
                send_command(arduino, 'S')
                
        else:  # direction == "right"
            # After right turn, front and left should be aligned
            front = filtered_rays.get("front", 0)
            left = filtered_rays.get("left", 0)
            
            if front == 0 or left == 0:
                fine_tune_iter += 1
                time.sleep(0.05)
                continue
            
            diff = abs(front - left)
            print(f"[FINE-TUNE] Front: {front:.1f}cm, Left: {left:.1f}cm, Diff: {diff:.1f}cm")
            
            if diff <= tolerance:
                send_command(arduino, 'S')
                print(f"[TURN] Complete. ICP: {total_theta:.2f}°, Fine-tune: {fine_tune_iter} steps")
                ray_buffer.clear()
                time.sleep(0.5)
                return True
            
            # Apply correction
            if front > left:
                # Turn left slightly
                correction_speed = min(30, int(15 + diff * 2))
                send_command(arduino, f'L{correction_speed}')
                time.sleep(0.08)
                send_command(arduino, 'S')
            else:
                # Turn right slightly
                correction_speed = min(30, int(15 + diff * 2))
                send_command(arduino, f'R{correction_speed}')
                time.sleep(0.08)
                send_command(arduino, 'S')
        
        # Check for convergence
        if abs(diff - last_diff) < 0.5:
            no_change_count += 1
            if no_change_count > 5:
                print(f"[FINE-TUNE] Converged at diff: {diff:.1f}cm")
                send_command(arduino, 'S')
                ray_buffer.clear()
                time.sleep(0.5)
                return True
        else:
            no_change_count = 0
            last_diff = diff
        
        time.sleep(0.05)
        fine_tune_iter += 1
    
    send_command(arduino, 'S')
    print(f"[TURN] Timeout after {max_fine_tune} fine-tune steps. ICP rotation: {total_theta:.2f}°")
    return False

def turn(laser, scan, arduino, direction, turn_time=0.7, tolerance_cm=1.5, timeout_sec=3):

    command = 'L65' if direction == "left" else 'R65'
    send_command(arduino, command)
    time.sleep(turn_time)
    send_command(arduino, 'S')
    time.sleep(0.3)
 
    ray_buffer.clear()
    print(f"[TURN] Aligning {direction}...")
    
    start_time = time.time()
    no_ray_count = 0
 
    while True:
        elapsed = time.time() - start_time
        if elapsed > timeout_sec:
            print(f"[TURN] Timeout after {timeout_sec}s")
            send_command(arduino, 'S')
            return False
 
        if get_fresh_scan(laser, scan) is None:
            time.sleep(0.05)
            continue
        
        rays = get_rays(scan)
        if not rays:
            time.sleep(0.05)
            continue
        
        ray_buffer.add(rays)
        rays = ray_buffer.get_median()
        if direction == "left":
            ray_a = rays["right_top_ray"]
            ray_b = rays["right_bot_ray"]
        else:
            ray_a = rays["left_pos_ray"]
            ray_b = rays["left_neg_ray"]
 
        if ray_a == 0 or ray_b == 0:
            no_ray_count += 1
            if no_ray_count > 10:
                print(f"[TURN] No wall detected on {direction} side")
                send_command(arduino, 'S')
                return False
            time.sleep(0.05)
            continue
        
        no_ray_count = 0
        diff = abs(ray_a - ray_b)
        
        if diff <= tolerance_cm:
            send_command(arduino, 'S')
            print(f"[TURN] Aligned. ray_a: {ray_a:.2f}cm, ray_b: {ray_b:.2f}cm")
            return True
        
        correction_speed = int(20 + (diff * 3))
        correction_speed = min(80, correction_speed)
        
        if ray_a > ray_b:
            send_command(arduino, f'R{correction_speed}')
        else:
            send_command(arduino, f'L{correction_speed}')
        time.sleep(0.1)

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

Tile = namedtuple('Tile', ['x', 'y'])

class TileState(IntEnum):
    UNVISITED = 0
    VISITED = 1
    BLACK = 2
    RED  = 3

class Direction(IntEnum):

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class MotorController:
    # moves the      robot
    def __init__(self, arduino, laser, scan):
        self.arduino = arduino
        self.laser = laser
        self.scan = scan
        self.x = 0          
        self.y = 0          
        self.heading = Direction.NORTH 
    def move_forward_one_tile(self):
        global ray_buffer
        ray_buffer.clear()
        if move_one_tile(self.laser, self.scan, self.arduino):
            # Update the position 
            if self.heading == Direction.NORTH:
                self.y += 1
            elif self.heading == Direction.EAST:
                self.x += 1
            elif self.heading == Direction.SOUTH:
                self.y -= 1
            elif self.heading == Direction.WEST:
                self.x -= 1
            return True
        return False
    def turn_relative(self, delta_heading: int):
        if delta_heading == 0:
            return True
        if delta_heading == 1:   # right
            success = turn(self.laser, self.scan, self.arduino, "right")
            if success:
                self.heading = Direction((self.heading + 1) % 4)
                
            return success
        elif delta_heading == -1: # left
            success = turn(self.laser, self.scan, self.arduino, "left")
            
            if success:
                self.heading = Direction((self.heading - 1) % 4)
            return success
        elif delta_heading == 2 or delta_heading == -2: # 180°
            # Two right turns
            if self.turn_relative(1) and self.turn_relative(1):
                
                return True
            return False
        return False
    def read_floor_sensor(self) :

        return ""
    
    def scan_for_victims(self) :
           
            return ""

    def get_position(self) :
        return (self.x, self.y)

    def get_heading(self) :
        return self.heading

    def set_heading(self, heading: Direction):
        self.heading = heading

class LidarAdapter:
    def __init__(self, laser, scan, get_heading_func):
        self.laser = laser
        self.scan = scan
        self.get_heading = get_heading_func

    def get_rays(self):
        if get_fresh_scan(self.laser, self.scan) is None:
            return {'N': 0, 'E': 0, 'S': 0, 'W': 0}
        rel = get_rays(self.scan)
        front = rel.get("front", 0)
        right = rel.get("right", 0)
        left = rel.get("left", 0)
        back = rel.get("back", 0)

        heading = self.get_heading()
        # Map relative directions
        if heading == Direction.NORTH:
            return {'N': front, 'E': right, 'S': back, 'W': left}
        elif heading == Direction.EAST:
            return {'N': left, 'E': front, 'S': right, 'W': back}
        elif heading == Direction.SOUTH:
            return {'N': back, 'E': left, 'S': front, 'W': right}
        else:  # WEST
            return {'N': right, 'E': back, 'S': left, 'W': front}
        
class DFSMazeExplorer:
    def __init__(self, width: int, height: int, start_x: int = 0, start_y: int = 0):
        self.width = width
        self.height = height
        self.start_x = start_x
        self.start_y = start_y
        
        self.map = [[TileState.UNVISITED for _ in range(height)] for _ in range(width)]
        self.walls = [[{'N': False, 'E': False, 'S': False, 'W': False} for _ in range(height)] for _ in range(width)]
        self.victims = [[None for _ in range(height)] for _ in range(width)]
        self.stack = []
        
        self.current_x = start_x
        self.current_y = start_y
        self.current_heading = Direction.NORTH
        
        self.exploration_complete = False
        self.total_tiles_visited = 0
        
        self.motor_controller = None
        self.lidar = None

    def set_hardware(self, motor_controller, lidar_adapter):
        self.motor_controller = motor_controller
        self.lidar = lidar_adapter

    def get_walls(self) :
        if not self.lidar:
            print("[WARNING] LiDAR not initialized")
            return {'N': False, 'E': False, 'S': False, 'W': False}
        
        wall_threshold = 28 
        rays = self.lidar.get_rays()
        
        wall_data = {
            'N': rays['N'] < wall_threshold,
            'E': rays['E'] < wall_threshold,
            'S': rays['S'] < wall_threshold,
            'W': rays['W'] < wall_threshold,
        }
        
        print(f"[WALLS] X={self.current_x}, Y={self.current_y} | {wall_data}")
        return wall_data
    
    def check_black_tile(self) :
        if not self.motor_controller:
            return False
        floor_color = self.motor_controller.read_floor_sensor()
        if floor_color == 'BLACK':
            print(f"[BLACK TILE] at ({self.current_x}, {self.current_y})")
            return True
        return False

    def check_silver_tile(self) :
        if not self.motor_controller:
            return False
        floor_color = self.motor_controller.read_floor_sensor()
        if floor_color == 'SILVER':
            print(f"[SILVER TILE] at ({self.current_x}, {self.current_y})")
            return True
        return False

    def scan_for_victims(self) :
        victim = self.motor_controller.scan_for_victims() if self.motor_controller else None
        if victim:
            print(f"[VICTIM FOUND] {victim} at ({self.current_x}, {self.current_y})")
            self.victims[self.current_x][self.current_y] = victim
        return victim

    def get_direction_offset(self, direction: Direction) :
        offsets = {
            Direction.NORTH: (0, 1),
            Direction.EAST: (1, 0),
            Direction.SOUTH: (0, -1),
            Direction.WEST: (-1, 0),
        }
        return offsets[direction]
    
    def get_wall_key_for_direction(self, direction: Direction) :
        keys = {
            Direction.NORTH: 'N',
            Direction.EAST: 'E',
            Direction.SOUTH: 'S',
            Direction.WEST: 'W',
        }
        return keys[direction]

    def get_unvisited_neighbors(self):
        unvisited = []
        for direction in Direction:
            dx, dy = self.get_direction_offset(direction)
            nx, ny = self.current_x + dx, self.current_y + dy
            wall_key = self.get_wall_key_for_direction(direction)
            if self.walls[self.current_x][self.current_y][wall_key]:
                continue
            if self.map[nx][ny] == TileState.UNVISITED:
                unvisited.append(direction)
        return unvisited

    def get_next_neighbor(self, unvisited):
        if not unvisited:
            return None
        
        # Convert integers to Direction enums to avoid type mismatch
        priority_order = [
            self.current_heading,
            Direction((self.current_heading + 1) % 4),
            Direction((self.current_heading + 3) % 4),
            Direction((self.current_heading + 2) % 4),
        ]
        
        for preferred_dir in priority_order:
            if preferred_dir in unvisited:
                return preferred_dir
        
        return unvisited[0]

    def move_to(self, target_x: int, target_y: int, target_heading: Direction) :
        if not self.motor_controller:
            return False
        
        # Determine relative turn needed
        delta = (target_heading - self.current_heading) % 4
        if delta == 0:
            # Same direction, just move forward
            return self.motor_controller.move_forward_one_tile()
        elif delta == 1:
            # Turn right then move
            if not self.motor_controller.turn_relative(1):
                return False
            return self.motor_controller.move_forward_one_tile()
        elif delta == 3:
            # Turn left then move
            if not self.motor_controller.turn_relative(-1):
                return False
            return self.motor_controller.move_forward_one_tile()
        elif delta == 2:
            # 180° turn then move
            if not self.motor_controller.turn_relative(2):
                return False
            return self.motor_controller.move_forward_one_tile()
        return False

    def mark_current_tile_visited(self):
        self.map[self.current_x][self.current_y] = TileState.VISITED
        self.total_tiles_visited += 1
        print(f"[VISITED] ({self.current_x}, {self.current_y}) | Total: {self.total_tiles_visited}")

    def backtrack(self) :
        if not self.stack:
            print("[BACKTRACK] Stack empty - exploration complete")
            return False
        
        prev_x, prev_y = self.stack.pop()
        print(f"[BACKTRACK] Moving to ({prev_x}, {prev_y})")
        
        dx = prev_x - self.current_x
        dy = prev_y - self.current_y
        if dx > 0:
            target_heading = Direction.EAST
        elif dx < 0:
            target_heading = Direction.WEST
        elif dy > 0:
            target_heading = Direction.NORTH
        else:
            target_heading = Direction.SOUTH
        
        if self.move_to(prev_x, prev_y, target_heading):
            self.current_x = prev_x
            self.current_y = prev_y
            self.current_heading = target_heading
            return True
        return False

    def exploration_step(self):
        global ray_buffer
        ray_buffer.clear()
        heading = self.current_heading
        if isinstance(heading, int):
            heading = Direction(heading)
            self.current_heading = heading
        print(f"\n{'='*60}")
        print(f"[STEP] Position: ({self.current_x}, {self.current_y}) | Heading: {self.current_heading.name}")
        print(f"{'='*60}")
        
        self.mark_current_tile_visited()
        
        # self.scan_for_victims()
        
        # if self.check_black_tile():
        #     self.map[self.current_x][self.current_y] = TileState.BLACK
        #     print("[ACTION] Black tile - backtrack")
        #     if not self.backtrack():
        #         self.exploration_complete = True
        #     return
        
        # if self.check_silver_tile():
        #     print("[CHECKPOINT] Silver tile - saving state (stub)")
        
        detected_walls = self.get_walls()
        self.walls[self.current_x][self.current_y] = detected_walls
        
        unvisited_neighbors = self.get_unvisited_neighbors()
        if unvisited_neighbors:
            next_dir = self.get_next_neighbor(unvisited_neighbors)
            print(f"[UNVISITED] {[d.name for d in unvisited_neighbors]} -> choose {next_dir.name}")
            
            self.stack.append((self.current_x, self.current_y))
            dx, dy = self.get_direction_offset(next_dir)
            next_x, next_y = self.current_x + dx, self.current_y + dy
            
            if self.move_to(next_x, next_y, next_dir):
                self.current_x = next_x
                self.current_y = next_y
                self.current_heading = next_dir
                
            else:
                self.stack.pop()
                print("[ERROR] Move failed")
        else:
            print("[DEAD END] No unvisited neighbors")
            if not self.backtrack():
                self.exploration_complete = True

    def run_exploration(self, max_steps: int = 1000):
        print(f"\n[START] DFS Exploration | Grid: {self.width}x{self.height} | Max steps: {max_steps}")
        step = 0
        while not self.exploration_complete and step < max_steps:
            try:
                self.exploration_step()
                step += 1
                time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n[INTERRUPT] Stopped by user")
                break
            except Exception as e:
                print(f"[CRITICAL ERROR] {e}")
                break
        print(f"\n[END] Steps: {step} | Tiles visited: {self.total_tiles_visited}")
        return self.exploration_complete

if __name__ == "__main__":
    arduino = None
    laser = None
    scan = None
    try:
        print("[START] Initializing robot...")
        arduino = init_arduino()
        laser = init_lidar()
        scan = ydlidar.LaserScan()
        motor = MotorController(arduino, laser, scan)
        lidar_adapter = LidarAdapter(laser, scan, motor.get_heading)
        
        explorer = DFSMazeExplorer(width=3, height=3, start_x=0, start_y=0)
        explorer.set_hardware(motor, lidar_adapter)
        
        explorer.run_exploration(max_steps=200)
        move_one_tile(laser , scan , arduino)
        # better_turn(laser , scan , arduino , "right")
        # turn_90(laser , scan , arduino , "right")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        print("[SHUTDOWN] Stopping robot...")
        if arduino:
            send_command(arduino, 'S')
            arduino.close()
        if laser:
            laser.turnOff()
            laser.disconnecting()
        print("[SHUTDOWN] Done")

