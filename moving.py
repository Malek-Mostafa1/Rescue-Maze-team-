import os
import ydlidar
import time
import math
import serial

# -------------------------
# SERIAL CONFIG
# -------------------------
SERIAL_PORT = "/dev/ttyUSB0"  # Arduino serial port
BAUDRATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUDRATE)
time.sleep(2)  

def send_serial(cmd):
    """Send command string to Arduino."""
    ser.write((cmd + "\n").encode())

# -------------------------
# DISTANCE FUNCTION
# -------------------------
def get_distances(scan):
    binned = {0: [], 90: [], -90: [], 180: []}

    for point in scan.points:
        angle = math.degrees(point.angle)
        dist = point.range * 100  

        if -5 <= angle <= 5:
            binned[0].append(dist)
        elif 85 <= angle <= 95:
            binned[90].append(dist)
        elif -95 <= angle <= -85:
            binned[-90].append(dist)
        elif angle >= 175 or angle <= -175:
            binned[180].append(dist)

    distances = {}
    for a in binned:
        if binned[a]:
            distances[a] = sum(binned[a]) / len(binned[a])
        else:
            distances[a] = 0
    return distances

# -------------------------
# MOVE FUNCTION
# -------------------------
def move_one_tile(laser, scan, distance_to_move=30):
    laser.doProcessSimple(scan)
    distances = get_distances(scan)
    front_distance = distances[-90]

    if front_distance == 0:
        print("No valid front distance detected. Aborting move.")
        return

    first_reading = front_distance
    target_distance = first_reading - distance_to_move

    # start moving
    send_serial("F")
    print(f"Moving forward. First reading: {first_reading:.2f} cm, target: {target_distance:.2f} cm")

    while True:
        laser.doProcessSimple(scan)
        distances = get_distances(scan)
        front_distance = distances[-90]

        if front_distance <= target_distance:
            break

        time.sleep(0.05)  # small delay to avoid busy loop

    send_serial("S")
    print(f"Movement complete. Final front distance: {front_distance:.2f} cm")

# -------------------------
# MAIN CODE
# -------------------------
if __name__ == "__main__":
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    lidar_port = "/dev/ydlidar"
    for _, value in ports.items():
        lidar_port = value
        break
    print("Using LiDAR port:", lidar_port)

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, lidar_port)
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

    scan = ydlidar.LaserScan()

    try:
        move_one_tile(laser, scan, distance_to_move=30)
    finally:
        laser.turnOff()
        laser.disconnecting()
        ser.close()