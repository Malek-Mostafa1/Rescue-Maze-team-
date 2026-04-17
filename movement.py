import time
import operator
from src.shared import movement_lock, ray_buffer
from src.arduino_handler import send_command
from src.lidar_handler import get_fresh_scan, get_rays
from src.imu_handler import get_current_yaw
from src.camera_handler import check_victim

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
    very_very_first_front_value_for_hole_detection = rays["front"]
    front_distance = rays["front"]
    right_distance = rays["right"]
    left_distance = rays["left"]

    if front_distance == 0 or front_distance < distance_to_move:
        print(" Front blocked")
        return False
 
    target_distance = max(front_distance - distance_to_move, 7)
    initial_right = ((right_distance + 15) % 30) - 15
    initial_left  = ((left_distance  + 15) % 30) - 15
    initial_offset = initial_right - initial_left
 
    base_speed = 60
    start_time = time.time()
    old_right_speed  = base_speed
    old_left_speed= base_speed

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

        modded_right_distance = ((right_distance + 15) % 30) - 15
        print(f'modded_right : {modded_right_distance}')
        modded_left_distance  = ((left_distance  + 15) % 30) - 15
        print(f'modded left : {modded_left_distance}')
        error = (modded_right_distance - modded_left_distance) - initial_offset
        print(f'intial {initial_offset}')
        print(f'error :{abs(error)}')
        g = operator.add
        j = operator.add  
        if abs(error) <= 1.2:
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
            
        k = 1.1

        right_speed = g(base_speed , (k * error))
        left_speed  = j(base_speed  , (k * error))

        right_speed = max(min(int(right_speed), 60), 40)
        left_speed  = max(min(int(left_speed), 60), 40)

        if right_speed != old_right_speed or left_speed != old_left_speed:
            send_command(arduino, f'F{right_speed},F{left_speed}')
        
        old_right_speed, old_left_speed = right_speed, left_speed
        
        print("moving")
        from src.color_handler import check_tile_color
        color = check_tile_color()
        print(color)
        if color == "black":
            send_command(arduino , "S")
            time.sleep(0.5)
            print("a hole has been spotted wow")
            ray_buffer.clear()
            time.sleep(0.05)
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

            initial_right = ((right_distance + 15) % 30) - 15
            initial_left  = ((left_distance  + 15) % 30) - 15
            initial_offset = initial_right - initial_left

            base_speed = 45
            start_time = time.time()
            old_right_speed = base_speed
            old_left_speed = base_speed
            send_command(arduino, f'B{base_speed},B{base_speed}')
            while True:
                ray_buffer.clear()
                time.sleep(0.05)

                elapsed = time.time() - start_time
                if elapsed > timeout_sec:
                    print(f"Timeout after {timeout_sec}s")
                    send_command(arduino, 'S')
                    time.sleep(0.01)
                    send_command(arduino , 'S')
                    return False

                if get_fresh_scan(laser, scan) is None:
                    print("[MOVE] Failed to get LiDAR reading")
                    send_command(arduino, 'S')
                    time.sleep(0.01)
                    send_command(arduino , 'S')
                    return False

                distances = get_rays(scan)
                if not distances:
                    print("[MOVE] Invalid ray data")
                    send_command(arduino, 'S')
                    time.sleep(0.01)
                    send_command(arduino , 'S')
                    return False

                ray_buffer.add(distances)
                rays = ray_buffer.get_median()

                front_raw = rays["front"]
                right_distance = rays["right"]
                left_distance = rays["left"]

                modded_right_distance = ((right_distance + 15) % 30) - 15
                modded_left_distance  = ((left_distance  + 15) % 30) - 15

                error = (modded_right_distance - modded_left_distance) - initial_offset

                print(f"modded_right: {modded_right_distance}")
                print(f"modded_left : {modded_left_distance}")
                print(f"initial_offset: {initial_offset}")
                print(f"error: {abs(error)}")

                g = operator.add
                j = operator.add

                if abs(error) <= 1.2:
                    error = 0
                    print("no change")
                elif abs(error) <= 6.1:
                    print("aligning left")
                    g = operator.sub
                    j = operator.add
                else:
                    print("aligning right")
                    g = operator.add
                    j = operator.sub

                k = 0.9
                right_speed = g(base_speed, (k * error))
                left_speed  = j(base_speed, (k * error))

                right_speed = max(min(int(right_speed), 45), 40)
                left_speed  = max(min(int(left_speed), 45), 40)

                if right_speed != old_right_speed or left_speed != old_left_speed:
                    send_command(arduino, f'B{right_speed},B{left_speed}')

                old_right_speed, old_left_speed = right_speed, left_speed

                print("moving backward")

                if front_raw >= very_very_first_front_value_for_hole_detection:
                    send_command(arduino, 'S')
                    time.sleep(0.01)
                    send_command(arduino , 'S')
                    print(f"Reached tile start: {front_raw:.2f}cm")
                    time.sleep(0.2)
                    ray_buffer.clear()
                    time.sleep(0.5)
                    return "hole"

        if front_raw <= 6:
            send_command(arduino, 'S')
            time.sleep(0.01)
            send_command(arduino , 'S')
            print("no space ")
            time.sleep(0.2)
            ray_buffer.clear()
            time.sleep(0.5)
            return True
        if front_raw <= target_distance:
            send_command(arduino, 'S')
            time.sleep(0.01)
            send_command(arduino , 'S')
            print(f" Reached tile: {front_raw:.2f}cm")
            time.sleep(0.2)
            ray_buffer.clear()
            time.sleep(0.05)
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

            initial_right = ((right_distance + 15) % 30) - 15
            initial_left  = ((left_distance  + 15) % 30) - 15
            if initial_left > initial_right :
                send_command(arduino , "R40")
                time.sleep(0.1)
                send_command(arduino , 'S')
            elif initial_right > initial_left :
                send_command(arduino , "L40")
                time.sleep(0.1)
                send_command(arduino , 'S')
            time.sleep(0.5)
            return True

def turn(ser, direction):
    with movement_lock:
        current_yaw = get_current_yaw()
        
        if direction.lower() == "right":
            target_yaw = current_yaw - 90
        else:
            target_yaw = current_yaw + 90
        
        if target_yaw > 180:
            target_yaw -= 360
        if target_yaw < -180:
            target_yaw += 360
        
        tolerance = 3
        max_turns = 400
        turn_count = 0
        
        while turn_count < max_turns:
            current_yaw = get_current_yaw()
            
            error = target_yaw - current_yaw
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            
            base_speed = 45
            if error > 0:  # Need to turn left
                left_speed = max(40, base_speed - int(abs(error) * 0.5))
                right_speed = base_speed
                send_command(ser, f'L{left_speed}')
                print(f"[TURN] Left - L:{left_speed} R:{right_speed} err:{error:.1f}")
            else:  # Need to turn right
                right_speed = max(40, base_speed - int(abs(error) * 0.5))
                left_speed = base_speed
                send_command(ser, f'R{right_speed}')
                print(f"[TURN] Right - L:{left_speed} R:{right_speed} err:{error:.1f}")
            
            if abs(error) <= tolerance:
                send_command(ser, "S")
                time.sleep(0.3)
                final_yaw = get_current_yaw()
                print(f"[TURN] {direction.upper()} complete! Final yaw: {final_yaw:.1f}°")
                return True
            
            time.sleep(0.02)
            turn_count += 1
        
        send_command(ser, "S")
        print(f"[TURN] Timeout")
        return False

class MotorController:
    def __init__(self, arduino, laser, scan):
        from src.dfs import Direction
        self.arduino = arduino
        self.laser = laser
        self.scan = scan
        self.x = 0          
        self.y = 0          
        self.heading = Direction.NORTH 
        self.Direction = Direction
    
    def move_forward_one_tile(self):
        ray_buffer.clear()
        success = move_one_tile(self.laser, self.scan, self.arduino)
        print("succcccccccccccccccccc",success)
        if success =="hole":
            return "hole"
        if success:
            if self.heading == self.Direction.NORTH:
                self.y += 1
            elif self.heading == self.Direction.EAST:
                self.x += 1
            elif self.heading == self.Direction.SOUTH:
                self.y -= 1
            elif self.heading == self.Direction.WEST:
                self.x -= 1
            return True
        return False
    
    def turn_relative(self, delta_heading: int):
        if delta_heading == 0:
            return True
        if delta_heading == 1:
            success = turn(self.arduino, "right")
            if success:
                self.heading = self.Direction((self.heading + 1) % 4)
            return success
        elif delta_heading == -1:
            success = turn(self.arduino, "left")
            if success:
                self.heading = self.Direction((self.heading - 1) % 4)
            return success
        elif delta_heading == 2 or delta_heading == -2:
            if self.turn_relative(1) and self.turn_relative(1):
                return True
            return False
        return False
    
    def read_floor_sensor(self):
        return ""
    
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

    def set_heading(self, heading):
        self.heading = heading
