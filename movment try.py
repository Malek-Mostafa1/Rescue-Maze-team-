import ydlidar
import time
import math


# arduino = serial.Serial('/dev/ttyACM0',115200)
# time.sleep(2)

ydlidar.os_init()

laser = ydlidar.CYdLidar()

laser.setlidaropt(ydlidar.LidarPropSerialPort,"/dev/ttyUSB1")
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate,230400)
laser.setlidaropt(ydlidar.LidarPropLidarType,ydlidar.TYPE_TOF)
laser.setlidaropt(ydlidar.LidarPropDeviceType,ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency,10.0)

scan = ydlidar.LaserScan()


def get_distance_90():

    if laser.doProcessSimple(scan):

        distances = []

        for p in scan.points:

            angle = p.angle * 180 / math.pi
            distance = p.range * 100   # meters → cm

            if 88 <= angle <= 92 and distance > 0:
                distances.append(distance)

        if len(distances) > 0:
            return sum(distances)/len(distances)

    return None



if laser.initialize():

    laser.turnOn()

    print("LiDAR Started")

   
    Distance_X = None

    while Distance_X is None:
        Distance_X = get_distance_90()

    print("Initial Distance:", Distance_X)

   
    Distance_Y = Distance_X - 30

    print("Target Distance:", Distance_Y)

    
    # arduino.write(b'F')

    while True:

        Distance_90 = get_distance_90()

        if Distance_90 is None:
            continue

        print("Current Distance:", Distance_90)

        if Distance_90 <= Distance_Y:

            # arduino.write(b'S')
            print("Target reached. Robot stopped.")
            break

        time.sleep(0.05)

    laser.turnOff()
    laser.disconnecting()

else:

    print("Failed to initialize LiDAR")