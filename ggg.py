import serial

# CHANGE this if needed
PORT = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

while True:
    try:
        line = ser.readline().decode('utf-8').strip()

        if not line:
            continue

        print("RAW:", line)


    except Exception as e:
        print("Error:", e)