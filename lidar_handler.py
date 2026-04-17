import math
import time
import ydlidar
from src.shared import lidar_lock

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

        if dist == 0 or dist > 1200:  # Changed from 200 to 1200 (12 meters)
            continue
        
        for name, target in target_rays.items():
            diff = abs(angle_deg - target)
            if diff < tolerance and diff < best[name]:
                best[name] = diff
                rays[name] = dist

    for name in rays:
        if rays[name] is None:
            rays[name] = 1200  # Changed fallback from 0 to 1200 so it defaults to open space
    
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

class LidarAdapter:
    def __init__(self, laser, scan, get_heading_func):
        from src.dfs import Direction
        self.laser = laser
        self.scan = scan
        self.get_heading = get_heading_func
        self.Direction = Direction

    def get_rays(self):
        if get_fresh_scan(self.laser, self.scan) is None:
            return {'N': 0, 'E': 0, 'S': 0, 'W': 0}
        rel = get_rays(self.scan)
        front = rel.get("front", 0)
        right = rel.get("right", 0)
        left = rel.get("left", 0)
        back = rel.get("back", 0)

        heading = self.get_heading()
        if heading == self.Direction.NORTH:
            return {'N': front, 'E': right, 'S': back, 'W': left}
        elif heading == self.Direction.EAST:
            return {'N': left, 'E': front, 'S': right, 'W': back}
        elif heading == self.Direction.SOUTH:
            return {'N': back, 'E': left, 'S': front, 'W': right}
        else:  # WEST
            return {'N': right, 'E': back, 'S': left, 'W': front}
