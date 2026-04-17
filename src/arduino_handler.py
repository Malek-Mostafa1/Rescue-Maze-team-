import time
import serial

def init_arduino(port='/dev/ttyUSB0', baudrate=115200):
    try:
        arduino = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        # Clear any pending data
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()
        print(f"[INIT] Arduino connected on {port}")
        return arduino
    except serial.SerialException as e:
        raise RuntimeError(f"Failed to connect to Arduino: {e}")

def send_command(arduino, command):
    if arduino:
        arduino.write((command + '\n').encode())
        print(f"[SERIAL] Sent: {command}")
