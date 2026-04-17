import time
import smbus
from src.shared import imu_lock

MPU_ADDR    = 0x68   
PWR_MGMT_1  = 0x6B   
GYRO_ZOUT_H = 0x47   

class IMUHandler:
    def __init__(self, calibration_samples=100):
        self.bus = smbus.SMBus(1) 
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        self.gyro_offset = 0
        self.yaw = 0
        self.prev_time = time.time()
        
        self.calibrate(calibration_samples)

    def calibrate(self, samples):
        offset = 0
        for _ in range(samples):
            high = self.bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H)
            low = self.bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H + 1)
            gz_raw = (high << 8) | low
            if gz_raw > 32768:
                gz_raw -= 65536
            gz = gz_raw / 131.0
            offset += gz
            time.sleep(0.01)
        self.gyro_offset = offset / samples
        print(f"[IMU] Gyro offset: {self.gyro_offset:.3f}")

    def read_word(self, reg):
        with imu_lock:
            high = self.bus.read_byte_data(MPU_ADDR, reg)
            low = self.bus.read_byte_data(MPU_ADDR, reg + 1)
            value = (high << 8) | low
            if value > 32768:
                value -= 65536
            return value

    def update_yaw(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt > 0.1:  # Limit dt to prevent spikes
            dt = 0.02
        elif dt < 0.001:
            dt = 0.01
        
        gz_raw = self.read_word(GYRO_ZOUT_H)
        gz = gz_raw / 131.0 - self.gyro_offset
        self.yaw += gz * dt
        
        # Keep yaw in range
        if self.yaw < -180:
            self.yaw += 360
        if self.yaw > 180:
            self.yaw -= 360
        
        self.prev_time = current_time
        return self.yaw

# Global IMU instance reference, initialized in main.py
_imu_instance = None

def init_imu():
    global _imu_instance
    try:
        _imu_instance = IMUHandler()
    except Exception as e:
        print(f"[IMU] Initialization failed: {e}")
        _imu_instance = None
    return _imu_instance

def get_current_yaw():
    if _imu_instance:
        return _imu_instance.update_yaw()
    return 0

