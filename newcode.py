import time
import math
import serial
import ydlidar
import numpy as np
import cv2
import pytesseract
from collections import deque, namedtuple
from enum import IntEnum
import smbus
import smbus2
import threading
import operator

start_time = time.perf_counter()

lidar_lock = threading.Lock()
imu_lock = threading.Lock()
movement_lock = threading.Lock()

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

def check_victim(laser, scan, arduino, side):
    ray_buffer.clear()
    time.sleep(0.2)
    if get_fresh_scan(laser, scan) is None:
        print("[VICTIM] Failed to get initial LiDAR reading")
        return False
 
    distances = get_rays(scan)
    if not distances:
        print("[VICTIM] Invalid ray data")
        return False
    ray_buffer.add(distances)
    rays = ray_buffer.get_median()
    right_distance = rays["right"]
    left_distance = rays["left"]
    
    if side == "right":
        if right_distance < 15:
            print(f"[VICTIM] Right side too close ({right_distance:.1f}cm)")
            return False
        x = 0
        flip = True
    else:
        if left_distance < 15:
            print(f"[VICTIM] Left side too close ({left_distance:.1f}cm)")
            return False
        x = 1
        flip = False
    
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    
    cap = cv2.VideoCapture(x)
    if not cap.isOpened():
        print(f"[VICTIM] Failed to open camera {x}")
        return False
        
    config = r'--oem 3 --psm 10 -c tessedit_char_whitelist=HSU --lang eng'
    
    start_victim_time = time.time()
    timeout = 5.0
    
    while time.time() - start_victim_time < timeout:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        if flip:
            frame = cv2.flip(frame, -1)
            
        # Resize for faster processing
        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Color detection
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = red_mask1 | red_mask2
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        red_pixels = np.sum(red_mask > 0)
        green_pixels = np.sum(green_mask > 0)
        yellow_pixels = np.sum(yellow_mask > 0)
        
        total_pixels = red_pixels + green_pixels + yellow_pixels
        
        if total_pixels > 100:  # Minimum pixel threshold
            if red_pixels > green_pixels and red_pixels > yellow_pixels and red_pixels > 200:
                print("[VICTIM] RED detected")
                send_command(arduino, "red")
                cap.release()
                cv2.destroyAllWindows()
                return True
            elif green_pixels > red_pixels and green_pixels > yellow_pixels and green_pixels > 200:
                print("[VICTIM] GREEN detected")
                send_command(arduino, "green")
                cap.release()
                cv2.destroyAllWindows()
                return True
            elif yellow_pixels > red_pixels and yellow_pixels > green_pixels and yellow_pixels > 200:
                print("[VICTIM] YELLOW detected")
                send_command(arduino, "yellow")
                cap.release()
                cv2.destroyAllWindows()
                return True
        
        # OCR for H, S, U
        try:
            # Preprocess for better OCR
            gray = cv2.medianBlur(gray, 3)
            gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            
            text = pytesseract.image_to_string(gray, config=config, timeout=1).strip().upper()
            if text and text[0] in ["H", "S", "U"]:
                print(f"[VICTIM] OCR: {text[0]}")
                send_command(arduino, text[0])
                cap.release()
                cv2.destroyAllWindows()
                return True
        except Exception as e:
            pass
        
        time.sleep(0.05)
    
    cap.release()
    cv2.destroyAllWindows()
    print("[VICTIM] No victim found after timeout")
    return False

def get_rays(scan, tolerance=2):
    target_rays = {
        "front": -90,
        "right": 0,
        "left": 180,
        "back": 90,
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

        if dist == 0 or dist > 200:  # Ignore invalid and too far readings
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

def flush_lidar_buffer(laser, scan, flushes=3):
    for _ in range(flushes):
        laser.doProcessSimple(scan)
        time.sleep(0.02)

def get_fresh_scan(laser, scan):
    with lidar_lock:
        flush_lidar_buffer(laser, scan, flushes=2)
        r = laser.doProcessSimple(scan)
        if not r:
            return None
        return scan

MPU_ADDR    = 0x68   
PWR_MGMT_1  = 0x6B   
GYRO_ZOUT_H = 0x47   

bus = smbus.SMBus(1) 
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

gyro_offset = 0
calibration_samples = 100

for i in range(calibration_samples):
    high = bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H)
    low = bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H + 1)
    gz_raw = (high << 8) | low
    if gz_raw > 32768:
        gz_raw -= 65536
    gz = gz_raw / 131.0
    gyro_offset += gz
    time.sleep(0.01)
gyro_offset /= calibration_samples
print(f"[IMU] Gyro offset: {gyro_offset:.3f}")

prev_time = time.time()
yaw = 0

def read_word(reg):
    with imu_lock:
        high = bus.read_byte_data(MPU_ADDR, reg)
        low = bus.read_byte_data(MPU_ADDR, reg + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

def update_yaw():
    global yaw, prev_time
    current_time = time.time()
    dt = current_time - prev_time
    
    if dt > 0.1:  # Limit dt to prevent spikes
        dt = 0.02
    elif dt < 0.001:
        dt = 0.01
    
    gz_raw = read_word(GYRO_ZOUT_H)
    gz = gz_raw / 131.0 - gyro_offset
    yaw += gz * dt
    
    # Keep yaw in range
    if yaw < -180:
        yaw += 360
    if yaw > 180:
        yaw -= 360
    
    prev_time = current_time
    return yaw


def move_one_tile(laser, scan, arduino, distance_to_move=29, timeout_sec=5):
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
        time.sleep(0.05)
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
        if abs(error) <= 1.1:
            error = 0
            print("no change")
 
        elif abs(error) <= 6.1:
            print("aligning left")
            g = operator.sub
            j = operator.add
        else:
            print("aliging right")
            g = operator.add
            j = operator.sub
            
        k = 0.7

        right_speed = g(base_speed , (k * error))
        left_speed  = j(base_speed  , (k * error))

        right_speed = max(min(int(right_speed), 60), 40)
        left_speed  = max(min(int(left_speed), 60), 40)

        send_command(arduino, f'F{right_speed},F{left_speed}')
        print("moving")

        if front_raw <= 6:
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
            return True

def turn(ser, direction):
    with movement_lock:
        
        # Get current yaw
        current_yaw = update_yaw()
        
        # Calculate target
        if direction.lower() == "right":
            target_yaw = current_yaw - 90
        else:
            target_yaw = current_yaw + 90
        
        # Normalize target
        if target_yaw > 180:
            target_yaw -= 360
        if target_yaw < -180:
            target_yaw += 360
        
        tolerance = 3
        max_turns = 400
        turn_count = 0

        while turn_count < max_turns:
            current_yaw = update_yaw()
            
            # Calculate error (shortest path)
            error = target_yaw - current_yaw
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            
            # Simple proportional control
            base_speed = 45
            if error > 0:  # Need to turn left
                left_speed = max(30, base_speed - int(abs(error) * 0.5))
                right_speed = base_speed
                send_command(ser, f'L{left_speed}')
                print(f"[TURN] Left - L:{left_speed} R:{right_speed} err:{error:.1f}")
            else:  # Need to turn right
                right_speed = max(30, base_speed - int(abs(error) * 0.5))
                left_speed = base_speed
                send_command(ser, f'R{right_speed}')
                print(f"[TURN] Right - L:{left_speed} R:{right_speed} err:{error:.1f}")
            
            # Check if we've reached target
            if abs(error) <= tolerance:
                send_command(ser, "S")
                time.sleep(0.3)
                final_yaw = update_yaw()
                print(f"[TURN] {direction.upper()} complete! Final yaw: {final_yaw:.1f}°")
                return True
            
            time.sleep(0.02)
            turn_count += 1
        
        send_command(ser, "S")
        print(f"[TURN] Timeout")
        return False

I2C_BUS  = 1
ADDR     = 0x29
CMD      = 0x80

BLACK_MAX    = 25       # Black is ~12
WHITE_MIN    = 155      # White is ~168-178
SILVER_MIN   = 110      # Silver is ~131-140
SILVER_MAX   = 150      
RED_MIN      = 0.50     # Red ratio is ~0.66
BLUE_MIN     = 0.40     # Blue ratio is ~0.43-0.44

bus = smbus2.SMBus(I2C_BUS)

bus.write_byte_data(ADDR, CMD | 0x00, 0x01)   
time.sleep(0.003)
bus.write_byte_data(ADDR, CMD | 0x00, 0x03)   

bus.write_byte_data(ADDR, CMD | 0x01, 0xFF)   
bus.write_byte_data(ADDR, CMD | 0x0F, 0x01)   
time.sleep(0.01)

print("TCS3472 ready. Detecting BLACK | RED | BLUE | WHITE | SILVER ...\n")

def read_word(reg):
    low  = bus.read_byte_data(ADDR, CMD | reg)
    high = bus.read_byte_data(ADDR, CMD | (reg + 1))
    return (high << 8) | low

def check_tile_color():
    try:
        while True:
            c = read_word(0x14)   # clear
            r = read_word(0x16)   # red
            g = read_word(0x18)   # green
            b = read_word(0x1A)   # blue

            total = r + g + b

            if total == 0:
                color = "UNKNOWN"
                
            elif c < BLACK_MAX:
                color = "BLACK"
                return "black"
            elif (r / total) >= RED_MIN and r > g and r > b:
                color = "RED"
                return "red"
            elif (b / total) >= BLUE_MIN and b > r and b > g:
                color = "blue"
                return "blue"
            elif c > WHITE_MIN:
                color = "WHITE"
                return "white"
            elif c > SILVER_MIN and c <= SILVER_MAX:
                color = "SILVER"
                return "silver"
            else:
                color = "UNKNOWN"

            print(f"C={c:5d}  R={r:5d}  G={g:5d}  B={b:5d}  → {color}")
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\nStopped.")
        bus.write_byte_data(ADDR, CMD | 0x00, 0x00)  
        bus.close()

def init_arduino():
    try:
        arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        time.sleep(2)
        # Clear any pending data
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()
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
    RED = 3

class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class MotorController:
    def __init__(self, arduino, laser, scan):
        self.arduino = arduino
        self.laser = laser
        self.scan = scan
        self.x = 0          
        self.y = 0          
        self.heading = Direction.NORTH 
    
    def move_forward_one_tile(self):
        ray_buffer.clear()
        success = move_one_tile(self.laser, self.scan, self.arduino)
        if success:
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
        if delta_heading == 1:
            success = turn(self.arduino, "right")
            if success:
                self.heading = Direction((self.heading + 1) % 4)
            return success
        elif delta_heading == -1:
            success = turn(self.arduino, "left")
            if success:
                self.heading = Direction((self.heading - 1) % 4)
            return success
        elif delta_heading == 2 or delta_heading == -2:
            # Two right turns instead of 180
            if self.turn_relative(1) and self.turn_relative(1):
                return True
            return False
        return False
    
    def scan_for_victims(self):
        victim_found = None
        print("[VICTIM] Checking right side...")
        if check_victim(self.laser, self.scan, self.arduino, "right"):
            victim_found = "victim"
        if not victim_found:
            print("[VICTIM] Checking left side...")
            if check_victim(self.laser, self.scan, self.arduino, "left"):
                victim_found = "victim"
        if victim_found:
            print(f"[VICTIM FOUND] at ({self.x}, {self.y})")
        else:
            print(f"[VICTIM] No victim at ({self.x}, {self.y})")
        return victim_found

    def get_position(self):
        return (self.x, self.y)

    def get_heading(self):
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
        if heading == Direction.NORTH:
            return {'N': front, 'E': right, 'S': back, 'W': left}
        elif heading == Direction.EAST:
            return {'N': left, 'E': front, 'S': right, 'W': back}
        elif heading == Direction.SOUTH:
            return {'N': back, 'E': left, 'S': front, 'W': right}
        else:  # WEST
            return {'N': right, 'E': back, 'S': left, 'W': front}
        
class BFSMazeExplorer:
    def __init__(self, width: int, height: int, start_x: int = 0, start_y: int = 0):
        self.width = width
        self.height = height
        self.start_x = start_x
        self.start_y = start_y
        
        # Dictionaries for dynamic grid (supports negative coordinates)
        self.map = {}
        self.walls = {}
        self.victims = {}
        self.colors = {}
        
        self._ensure_tile_exists(start_x, start_y)
        
        self.current_x = start_x
        self.current_y = start_y
        self.current_heading = Direction.NORTH
        
        # Used for reverting a black tile encounter
        self.prev_x = start_x
        self.prev_y = start_y
        
        self.exploration_complete = False
        self.total_tiles_visited = 0
        
        self.motor_controller = None
        self.lidar = None

    def set_hardware(self, motor_controller, lidar_adapter):
        self.motor_controller = motor_controller
        self.lidar = lidar_adapter
    
    def _ensure_tile_exists(self, x, y):
        if x not in self.map:
            self.map[x] = {}
            self.walls[x] = {}
            self.victims[x] = {}
            self.colors[x] = {}
        if y not in self.map[x]:
            self.map[x][y] = TileState.UNVISITED
            self.walls[x][y] = {'N': False, 'E': False, 'S': False, 'W': False}
            self.victims[x][y] = None
            self.colors[x][y] = None
    
    def _is_visited(self, x, y):
        self._ensure_tile_exists(x, y)
        return self.map[x][y] == TileState.VISITED

    def get_walls(self):
        if not self.lidar:
            return {'N': False, 'E': False, 'S': False, 'W': False}
        
        wall_threshold = 27  # Slightly less than tile size for safety
        rays = self.lidar.get_rays()
        
        wall_data = {
            'N': (rays['N'] > 5) and (rays['N'] < wall_threshold),
            'E': (rays['E'] > 5) and (rays['E'] < wall_threshold),
            'S': (rays['S'] > 5) and (rays['S'] < wall_threshold),
            'W': (rays['W'] > 5) and (rays['W'] < wall_threshold),
        }
        
        print(f"[WALLS] ({self.current_x},{self.current_y}) heading={self.current_heading.name}")
        print(f"[WALLS] Distances - N:{rays['N']:.1f} E:{rays['E']:.1f} S:{rays['S']:.1f} W:{rays['W']:.1f}")
        print(f"[WALLS] Detected - {wall_data}")
        return wall_data
    
    def check_tile_color(self):
        color = check_tile_color()
        if color:
            print(f"[COLOR] Detected {color} at ({self.current_x}, {self.current_y})")
        return color

    def scan_for_victims(self):
        victim = self.motor_controller.scan_for_victims() if self.motor_controller else None
        if victim:
            self.victims[self.current_x][self.current_y] = victim
        return victim

    def get_direction_offset(self, direction: Direction):
        offsets = {
            Direction.NORTH: (0, 1),
            Direction.EAST: (1, 0),
            Direction.SOUTH: (0, -1),
            Direction.WEST: (-1, 0),
        }
        return offsets[direction]
    
    def get_wall_key_for_direction(self, direction: Direction):
        keys = {
            Direction.NORTH: 'N',
            Direction.EAST: 'E',
            Direction.SOUTH: 'S',
            Direction.WEST: 'W',
        }
        return keys[direction]

    def get_open_neighbors(self, x, y):
        neighbors = []
        if x not in self.walls or y not in self.walls[x]: 
            return neighbors
            
        walls = self.walls[x][y]
        for direction in Direction:
            wall_key = self.get_wall_key_for_direction(direction)
            if not walls[wall_key]: # No wall blocking
                dx, dy = self.get_direction_offset(direction)
                nx, ny = x + dx, y + dy
                self._ensure_tile_exists(nx, ny)
                # Never traverse black tiles
                if self.map[nx][ny] != TileState.BLACK:
                    neighbors.append((nx, ny, direction))
        return neighbors

    def find_path_to_nearest_unvisited(self):
        """BFS over the visited graph to find the shortest path to an unvisited tile."""
        queue = deque()
        start = (self.current_x, self.current_y)
        queue.append((start, []))
        visited_search = set([start])

        while queue:
            (cx, cy), path = queue.popleft()

            self._ensure_tile_exists(cx, cy)
            
            # If we reached an UNVISITED tile, return the path that got us here
            if self.map[cx][cy] == TileState.UNVISITED:
                return path

            # If it's a VISITED tile, we can continue traversing through it
            if self.map[cx][cy] == TileState.VISITED:
                for nx, ny, d in self.get_open_neighbors(cx, cy):
                    if (nx, ny) not in visited_search:
                        visited_search.add((nx, ny))
                        queue.append(((nx, ny), path + [(nx, ny, d)]))
        return None

    def move_to(self, target_x: int, target_y: int, target_heading: Direction):
        if not self.motor_controller:
            return False
        
        # Calculate relative turn needed
        delta = (target_heading - self.current_heading) % 4
        
        if delta == 0:
            return self.motor_controller.move_forward_one_tile()
        elif delta == 1:
            if not self.motor_controller.turn_relative(1): return False
            return self.motor_controller.move_forward_one_tile()
        elif delta == 3:
            if not self.motor_controller.turn_relative(-1): return False
            return self.motor_controller.move_forward_one_tile()
        elif delta == 2:
            if not self.motor_controller.turn_relative(2): return False
            return self.motor_controller.move_forward_one_tile()
        return False

    def mark_current_tile_visited(self):
        self._ensure_tile_exists(self.current_x, self.current_y)
        self.map[self.current_x][self.current_y] = TileState.VISITED
        self.total_tiles_visited += 1
        print(f"[VISITED] ({self.current_x},{self.current_y}) total={self.total_tiles_visited}")

    def step_back(self):
        """Handle retreat from a black tile by returning to the exact previous tile."""
        print(f"[BACKTRACK] Returning to ({self.prev_x},{self.prev_y}) from ({self.current_x},{self.current_y})")
        dx = self.prev_x - self.current_x
        dy = self.prev_y - self.current_y
        
        if dx == 0 and dy == 0:
            print("[ERROR] Start tile is black, nowhere to step back to!")
            return False
            
        if dx > 0: target_heading = Direction.EAST
        elif dx < 0: target_heading = Direction.WEST
        elif dy > 0: target_heading = Direction.NORTH
        elif dy < 0: target_heading = Direction.SOUTH
        else: target_heading = self.current_heading
        
        if self.move_to(self.prev_x, self.prev_y, target_heading):
            self.current_x = self.prev_x
            self.current_y = self.prev_y
            self.current_heading = target_heading
            return True
        return False

    def exploration_step(self):
        ray_buffer.clear()
        
        if isinstance(self.current_heading, int):
            self.current_heading = Direction(self.current_heading)
        
        print(f"\n{'='*60}")
        print(f"[STEP] Position: ({self.current_x}, {self.current_y}) Heading: {self.current_heading.name}")
        print(f"{'='*60}")
        
        # Check tile color FIRST (before marking visited)
        color = self.check_tile_color()
        self.colors[self.current_x][self.current_y] = color
        
        if color == "black":
            print(f"[COLOR] BLACK tile detected - marking unreachable and reversing")
            self.map[self.current_x][self.current_y] = TileState.BLACK
            if not self.step_back():
                self.exploration_complete = True
            return # Skip scanning and pathfinding this tick so we can re-eval from safe tile
            
        elif color == "blue":
            print(f"[COLOR] BLUE tile detected - pausing 5 seconds")
            send_command(self.motor_controller.arduino, 'S')
            time.sleep(5)
            print(f"[COLOR] Resuming after blue tile pause")
        elif color == "red":
            print(f"[COLOR] RED tile detected - no action, continuing")
        elif color == "silver":
            print(f"[COLOR] SILVER checkpoint detected at ({self.current_x}, {self.current_y})")
        
        # Mark current tile as visited
        self.mark_current_tile_visited()
        
        # Check for victims
        self.scan_for_victims()
        
        # Detect walls
        detected_walls = self.get_walls()
        self.walls[self.current_x][self.current_y] = detected_walls
        
        # Calculate BFS to the absolute nearest unvisited tile
        path = self.find_path_to_nearest_unvisited()
        
        if path:
            # path[0] holds the next immediate step to get towards the goal
            next_x, next_y, next_dir = path[0]
            print(f"[BFS] Next step along route: move {next_dir.name} to ({next_x}, {next_y})")
            
            # Save our current location as prev before making the physical move
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            
            # Execute step
            if self.move_to(next_x, next_y, next_dir):
                self.current_x = next_x
                self.current_y = next_y
                self.current_heading = next_dir
            else:
                print("[ERROR] Move failed, staying at current tile")
        else:
            print("[BFS] No reachable unvisited tiles found. Maze is completely explored.")
            self.exploration_complete = True

    def run_exploration(self, max_steps: int = 1000):
        print(f"\n[START] BFS Exploration")
        print(f"[START] Starting position: ({self.start_x}, {self.start_y}) facing NORTH")
        print(f"[START] Grid expands dynamically, supports negative coordinates")
        
        step = 0
        while not self.exploration_complete and step < max_steps:
            try:
                self.exploration_step()
                step += 1
                time.sleep(0.2)  # Small delay between steps
            except KeyboardInterrupt:
                print("\n[INTERRUPT] Stopped by user")
                break
            except Exception as e:
                print(f"[ERROR] {e}")
                import traceback
                traceback.print_exc()
                break
        
        print(f"\n[END] Steps: {step} | Tiles visited: {self.total_tiles_visited}")
        print(f"[END] Final position: ({self.current_x}, {self.current_y})")
        return self.exploration_complete
    

def get_all_unvisited(self):
        """Scans the map dictionary and returns a list of all UNVISITED coordinates."""
        unvisited = []
        for x in self.map:
            for y in self.map[x]:
                if self.map[x][y] == TileState.UNVISITED:
                    unvisited.append((x, y))
        return unvisited

def calculate_path_cost(self, path, starting_heading):
    """Calculates physical driving cost: 1 per tile + 1 per 90-degree turn."""
    if not path:
        return float('inf')
        
    cost = 0
    current_heading = starting_heading
    
    for nx, ny, target_dir in path:
        # Calculate turn penalty (0=Straight, 1=Left/Right, 2=U-Turn)
        turn_diff = (target_dir - current_heading) % 4
        turn_cost = min(turn_diff, 4 - turn_diff)
        
        # Add 1 for the forward movement, plus the turn penalty
        cost += (1 + turn_cost) 
        current_heading = target_dir
        
    return cost
def _bfs_to_target(self, target_x, target_y):
    """Standard BFS that calculates the shortest grid path to a specific tile."""
    queue = deque()
    start = (self.current_x, self.current_y)
    queue.append((start, []))
    visited_search = set([start])
    while queue:
        (cx, cy), path = queue.popleft()
        if cx == target_x and cy == target_y:
            return path
        # Only traverse through VISITED tiles (or the starting tile itself)
        if self.map[cx][cy] == TileState.VISITED or (cx, cy) == start:
            for nx, ny, direction in self.get_open_neighbors(cx, cy):
                if (nx, ny) not in visited_search:
                    visited_search.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny, direction)]))
    return None
def find_path_to_nearest_unvisited(self):
    """Brute-forces the turn-weighted cost to every known unvisited tile and picks the best."""
    unvisited_tiles = self.get_all_unvisited()
    
    if not unvisited_tiles:
        return None
    best_path = None
    lowest_cost = float('inf')
    for tx, ty in unvisited_tiles:
        # Get the raw tile path
        path = self._bfs_to_target(tx, ty)
        
        if path:
            # Calculate what it actually costs the hardware to drive it
            cost = self.calculate_path_cost(path, self.current_heading)
            
            # Keep the cheapest one
            if cost < lowest_cost:
                lowest_cost = cost
                best_path = path
    return best_path

if __name__ == "__main__":
    arduino = None
    laser = None
    scan = None
    try:
        print("[START] Initializing robot...")
        arduino = init_arduino()
        laser = init_lidar()
        scan = ydlidar.LaserScan()
        
        # Wait for sensors to stabilize
        time.sleep(2)
        
        motor = MotorController(arduino, laser, scan)
        lidar_adapter = LidarAdapter(laser, scan, motor.get_heading)
        
        # Create BFS explorer with dynamic grid
        explorer = BFSMazeExplorer(width=20, height=20, start_x=0, start_y=0)
        explorer.set_hardware(motor, lidar_adapter)
        
        print("[DEBUG] Dynamic grid - supports negative coordinates")
        print("[DEBUG] Starting exploration...")
        
        explorer.run_exploration(max_steps=200)
        
    except Exception as e:
        print(f"[FATAL] {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[SHUTDOWN] Stopping robot...")
        if arduino:
            send_command(arduino, 'S')
            time.sleep(0.5)
            arduino.close()
        if laser:
            laser.turnOff()
            laser.disconnecting()
        print("[SHUTDOWN] Done")