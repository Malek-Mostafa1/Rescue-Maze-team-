import time
from lidar import initialize_lidar
from robot_serial import setup_arduino, send_command
from movement import move_one_tile, turn

def run_robot():
    arduino = None
    laser = None
    
    try:
        # Initialize hardware
        arduino = setup_arduino(port='/dev/ttyUSB1', baudrate=115200, timeout=1)
        laser, scan = initialize_lidar()

        # Core logic
        # Example operations:
        # x_dirs = check_directions(scan, threshold_cm=30)
        # while not x_dirs["front"]:
        #     send_command(arduino, "f60")

        move_one_tile(laser, scan, arduino, distance_to_move=30)
        
        # turn(laser, scan, arduino, "right")
        # move_one_tile(laser, scan, arduino, distance_to_move=30)

    except RuntimeError as e:
        print(f"Error initializing hardware: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
    finally:
        print("Cleaning up connections...")
        if laser is not None:
            laser.turnOff()
            laser.disconnecting()
        if arduino is not None:
            if arduino.is_open:
                arduino.close()

if __name__ == "__main__":
    # Entry point
    run_robot()
