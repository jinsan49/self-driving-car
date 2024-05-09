#!/usr/bin/env python
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import time
import math
import numpy as np
import matplotlib.pyplot as plt

# LiDAR data
scan_data = None
def scan_callback(data):
    global scan_data
    scan_data = data

def main():
    rclpy.init()
    node = rclpy.create_node('lidar_view')
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    # Wait until we receive the first LiDAR message
    while rclpy.ok():    
        if scan_data is None:
            print("Not receiving lidar data")
        else:
            break
    print("Receiving lidar data")
    
    # Main loop
    while rclpy.ok():
        range_array = np.array(scan_data.ranges, copy=True)
        front_distances = (np.append(range_array[-2:], range_array[:3]) * 100).astype(int)

        # min is np.min(front_distances)

        # for dist in front_distances:
        #     dist = front_distances * 100 

        print(front_distances)

        distances = range_array[len(range_array) - 2: 2] #gathering 0 degree to -about 15 degree

        # print('throttle: %d, steering: %d, auto_mode: %r' % (auto_throttle, auto_steer, auto))


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()