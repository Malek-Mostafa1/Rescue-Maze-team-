import time
from collections import namedtuple
from enum import IntEnum
from src.shared import ray_buffer
from src.arduino_handler import send_command
from src.movement import move_one_tile
from src.shared import movement_lock, ray_buffer
from src.arduino_handler import send_command
from src.lidar_handler import get_fresh_scan, get_rays
from src.imu_handler import get_current_yaw
from src.camera_handler import check_victim
from src.movement import turn

def right_wall_follower(laser , scan , arduino ):
    while True :
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
        front_distancce = rays["front"]
        right_distance = rays["right"]
        left_distance = rays["left"]

        if right_distance >= 25 : 
            turn("right")
        elif right_distance <= 15 and front_distancce <= 15 and left_distance >=25 :
            turn("left")
        else :
            move_one_tile(laser , scan , arduino)
            if move_one_tile == "hole" :
                turn("left")
                move_one_tile(laser , scan , arduino)
