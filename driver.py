#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import struct
import can
import threading
import time

throttle = 0
steer = 0
button = 0
prev_button = 0
auto = False
auto_throttle = 0
auto_steer = 0

## ------------
## YOUR CODE
## ------------
def throttle_callback(msg):
    global throttle
    throttle = msg.data

def steering_callback(msg):
    global steer
    steer = msg.data

def button_callback(msg):
    global button, prev_button, auto
    prev_button = button
    button = msg.data
    if button == 0 and prev_button == 1:
        auto = not(auto)

def auto_throttle_callback(msg):
    global auto_throttle
    auto_throttle = msg.data

def auto_steering_callback(msg):
    global auto_steer
    auto_steer = msg.data

def main(args=None):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    ## ------------
    ## YOUR CODE
    # Subscribe to the manual_throttle and manual_steering commands
    ## ------------
    node.create_subscription(Int64, 'manual_throttle', throttle_callback, 5)
    node.create_subscription(Int64, 'manual_steering', steering_callback, 5)
    # node.create_subscription(Int64, 'cv_throttle', auto_throttle_callback, 5)
    node.create_subscription(Int64, 'yolo_throttle', auto_throttle_callback, 5)
    node.create_subscription(Int64, 'cv_steer', auto_steering_callback, 5)
    node.create_subscription(Int64, 'auto_button', button_callback, 5)

    thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        try:
            # DO NOT COMPUTE THE PWM VALUES IN ORIN. Just send the raw command values. 
            if not auto:
                can_data = struct.pack('>hhI', throttle, steer, 0)
                print('throttle: %d, steering: %d, auto_mode: %r' % (throttle, steer, auto))
            else:
                can_data = struct.pack('>hhI', auto_throttle, auto_steer, 0)
                print('throttle: %d, steering: %d, auto_mode: %r' % (auto_throttle, auto_steer, auto))

            ## ------------
            ## YOUR CODE
            # Create a CAN message with can_data and  arbitration_id 0x1, and send it.
            # Hint: See Lecture 3 slides
            ## ------------
            msg = can.Message(arbitration_id=0x1, data=can_data, is_extended_id = False)
            ret = bus.send(msg)

        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()