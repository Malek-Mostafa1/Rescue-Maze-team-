import time
import ydlidar

from src.arduino_handler import init_arduino
from src.lidar_handler import init_lidar, LidarAdapter
from src.imu_handler import init_imu
from src.color_handler import init_color_sensor
from src.movement import MotorController
from src.greedy import GreedyMazeExplorer

def main():
    arduino = None
    laser = None
    scan = None
    imu = None
    color_sensor = None
    
    try:
        print("[START] Initializing sensors...")
        
        # Initialize components
        imu = init_imu()
        color_sensor = init_color_sensor()
        arduino = init_arduino()
        laser = init_lidar()
        
        scan = ydlidar.LaserScan()
        
        # Wait for sensors to stabilize
        print("[WAIT] letting sensors stabilize...")
        time.sleep(2)
        
        motor = MotorController(arduino, laser, scan)
        lidar_adapter = LidarAdapter(laser, scan, motor.get_heading)
        
        # Create explorer with dynamic grid
        explorer = GreedyMazeExplorer(width=10, height=10, start_x=0, start_y=0)
        explorer.set_hardware(motor, lidar_adapter)
        
        print("[DEBUG] Dynamic grid - supports negative coordinates")
        print("[DEBUG] Starting exploration...")
        
        explorer.run_exploration(max_steps=200)
    
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopped by user. Cleaning up...")
    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[CLEANUP] Closing connections...")
        if laser:
            laser.turnOff()
            laser.disconnecting()
        if arduino:
            arduino.close()
        if color_sensor:
            color_sensor.close()

if __name__ == "__main__":
    main()
