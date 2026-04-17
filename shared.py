import threading
import time
from collections import deque

lidar_lock = threading.Lock()
imu_lock = threading.Lock()
movement_lock = threading.Lock()

start_time = time.perf_counter()

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
            if rays.get(name, 0) != 0:
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
