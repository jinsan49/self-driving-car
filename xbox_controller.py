#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import threading
import curses

stdscr = curses.initscr()

# Throttle should be bounded between [-10, +4]
MAX_MANUAL_THROTTLE_FORWARD = 20
MAX_MANUAL_THROTTLE_REVERSE = 20

# Steering should be bounded between [-100, +100]

steering = 0
throttle = 0
button = 0

def joy_callback(data):
	global steering, throttle, button

	# print(data)	# delete this line once this callback is implemented
	steering = int(data.axes[2] * -100)
	throttle = int(data.axes[1] * 100)
	button = int(data.buttons[0])

	if throttle > MAX_MANUAL_THROTTLE_FORWARD:
		throttle = MAX_MANUAL_THROTTLE_FORWARD
	elif throttle < -MAX_MANUAL_THROTTLE_REVERSE:
		throttle = -MAX_MANUAL_THROTTLE_REVERSE



	

def main(args=None):

	rclpy.init(args=args)
	node = Node("xbox_controller_node")

	## ------------
	## YOUR CODE
	# Subscribe to the 'joy' topic
	# Create publishers for the manual_throttle and manual_steering commands
	## ------------
	node.create_subscription(Joy, '/joy', joy_callback, 5)

	manual_throttle_pub = node.create_publisher(Int64, 'manual_throttle', 10)
	manual_steering_pub = node.create_publisher(Int64, 'manual_steering', 10)
	auto_button_pub = node.create_publisher(Int64, 'auto_button', 10)

	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
        
	while rclpy.ok():

		try:
			## ------------
			## YOUR CODE
			# Publish actuation commands
			## ------------
			steer_msg = Int64()
			throttle_msg = Int64()
			steer_msg.data = steering
			throttle_msg.data = throttle
			manual_throttle_pub.publish(throttle_msg)
			manual_steering_pub.publish(steer_msg)

			button_msg = Int64()
			button_msg.data = button
			auto_button_pub.publish(button_msg)

			stdscr.refresh()
			stdscr.addstr(1, 25, 'Xbox Controller       ')
			stdscr.addstr(2, 25, 'Throttle: %.2f  ' % throttle)
			stdscr.addstr(3, 25, 'Steering: %.2f  ' % steering)
			stdscr.addstr(4, 25, 'Button: %.2f  ' % button)

			rate.sleep()
		except KeyboardInterrupt:
			curses.endwin()
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
