import time
from lidar import get_rays
from robot_serial import send_command

def check_directions(scan, threshold_cm=30):
    distances = get_rays(scan)
    front = distances.get("front", 0)
    right = distances.get("right", 0)
    back  = distances.get("back", 0)
    left  = distances.get("left", 0)
    return {
        "front": 0 < front <= threshold_cm,
        "right": 0 < right <= threshold_cm,
        "back":  0 < back  <= threshold_cm,
        "left":  0 < left  <= threshold_cm,
    }

def move_one_tile(laser, scan, arduino, distance_to_move=30):
    # Get initial front distance
    r = laser.doProcessSimple(scan)
    if not r:
        print("Failed to get initial LiDAR reading. Aborting move.")
        return

    distances = get_rays(scan)
    front_distance = distances["front"]

    if front_distance == 0:
        print("No valid front distance detected. Aborting move.")
        return

    target_distance = front_distance - distance_to_move
    print(f"Starting move: front={front_distance:.2f}cm, target={target_distance:.2f}cm")
    send_command(arduino, 'F80')

    while True:
        r = laser.doProcessSimple(scan)
        if not r:
            print("Failed to get LiDAR data. Stopping.")
            send_command(arduino, 'S')
            return

        distances = get_rays(scan)
        front_distance = distances["front"]

        if front_distance == 0:
            print("Lost front reading during movement. Stopping.")
            send_command(arduino, 'S')
            return

        print(f"Moving... front={front_distance:.2f}cm, target={target_distance:.2f}cm")
        if front_distance <= target_distance:
            send_command(arduino, 'S')
            print(f"Reached target. Final distance: {front_distance:.2f}cm")
            return

        time.sleep(0.05)

def turn(laser, scan, arduino, direction, turn_time=0.6, tolerance_cm=1.5):
    command = 'L80' if direction == "left" else 'R80'
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
