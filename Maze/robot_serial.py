import time
import serial

def setup_arduino(port="/dev/ttyUSB1", baudrate=115200, timeout=1):
    # Setup serial communication with Arduino
    try:
        arduino = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        print(f"Arduino connected on {port}")
        return arduino
    except serial.SerialException as e:
        raise RuntimeError(f"Failed to connect to Arduino on {port}: {e}")

def send_command(arduino, command):
    arduino.write(command.encode())
    print(f"[SERIAL] Sent: {command}")
