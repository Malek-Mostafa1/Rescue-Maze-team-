import math
import ydlidar

def setup(port):
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
    return laser

def initialize_lidar():
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for key, value in ports.items():
        port = value
        print(f"LiDAR port found: {port}")

    laser = setup(port)

    ret = laser.initialize()
    if not ret:
        raise RuntimeError("Failed to initialize LiDAR")

    ret = laser.turnOn()
    if not ret:
        laser.disconnecting()
        raise RuntimeError("Failed to turn on LiDAR")

    scan = ydlidar.LaserScan()
    return laser, scan

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
   # print(rays)
    return rays
