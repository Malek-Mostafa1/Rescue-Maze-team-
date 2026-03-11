import os
import ydlidar
import time
import math
   
if __name__ == "__main__":
    ydlidar.os_init();
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
        print(port);
    laser = ydlidar.CYdLidar();
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)

    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 4);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02);
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan();
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            if r:
                target_angles = [0, 90, -90, 180]
                binned = {a: [] for a in target_angles}

                for point in scan.points:
                    angle_deg = math.degrees(point.angle)
                    dist = point.range * 100

                    if -5 <= angle_deg <= 5:
                        binned[0].append(dist)

                    elif 85 <= angle_deg <= 95:
                        binned[90].append(dist)

                    elif -95 <= angle_deg <= -85:
                        binned[-90].append(dist)

                    elif angle_deg >= 175 or angle_deg <= -175:
                        binned[180].append(dist)

                # now compute average range for each bin
                for angle, ranges in binned.items():
                    if ranges:
                        avg_range = sum(ranges) / len(ranges)
                    else:
                        avg_range = 0
                    print("Angle {}^deg -> distance {:.3f} cm".format(angle, avg_range))
            else :
                print("Failed to get lidar data")
                time.sleep(0.05);
            time.sleep(0.05);
        laser.turnOff();
    laser.disconnecting();