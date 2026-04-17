import time
import cv2
import numpy as np
import pytesseract
from src.shared import ray_buffer
from src.lidar_handler import get_fresh_scan, get_rays
from src.arduino_handler import send_command

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


def scan_for_victims_while_moving(arduino, cameras, rays):
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    
    for side in ['left', 'right']:
        if rays[side] > 15:
            continue
            
        cap = cameras[side]
        ret, frame = cap.read()
        if not ret:
            continue

        if side == 'right':
            frame = cv2.flip(frame, -1)
            
        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        r_px = np.sum(red_mask > 0)
        g_px = np.sum(green_mask > 0)
        y_px = np.sum(yellow_mask > 0)

        black_pixels = np.sum(gray < 60)
        black_ratio = black_pixels / gray.size

        if max(r_px, g_px, y_px) > 200 or black_ratio > 0.04:
            send_command(arduino, "S")
            print(f"[VICTIM] {side.upper()} side triggered! Stopping robot.")
            return True
        else:
            return False