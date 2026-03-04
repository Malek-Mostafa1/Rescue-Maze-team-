import serial
import time
import math

ser = serial.Serial("COM5", 230400, timeout=1)

# start scan
ser.write(b'\xA5\x60')
time.sleep(0.2)

def read_block():
    while True:
        b1 = ser.read(1)
        if not b1:
            continue

        if b1[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55:
                payload = ser.read(128)
                if len(payload) == 128:
                    return payload


last_print = time.time()

while True:

    pkt = read_block()

    n = (pkt[0] << 8) | pkt[1]

    if n != 40:
        continue

    # angles are sent as deg * 100
    start_angle = ((pkt[2] << 8) | pkt[3]) / 100.0
    end_angle   = ((pkt[4] << 8) | pkt[5]) / 100.0

    # handle wrap (for example 350 -> 10)
    if end_angle < start_angle:
        end_angle += 360.0

    step = (end_angle - start_angle) / (n - 1)

    front = []
    left  = []
    right = []
    back  = []

    offset = 8

    for i in range(n):

        d = pkt[offset] | (pkt[offset + 1] << 8)
        offset += 3

        ang = start_angle + i * step
        ang = ang % 360.0

        # sectoring (±45°)
        if ang <= 45 or ang >= 315:
            front.append(d)
        elif 45 < ang <= 135:
            left.append(d)
        elif 135 < ang <= 225:
            back.append(d)
        else:
            right.append(d)

    now = time.time()

    if now - last_print >= 5.0:
        last_print = now

        def avg(lst):
            return int(sum(lst)/len(lst)) if lst else None

        print("Front:", avg(front),
              " Left:", avg(left),
              " Right:", avg(right),
              " Back:", avg(back))