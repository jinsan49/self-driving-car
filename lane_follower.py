#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
from std_msgs.msg import Int64
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = "/zed/zed_node/stereo/image_rect_color"

br = CvBridge()
image = None
last_cte = 0

# ZED Camera info
def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]

# Lane following algorithm
def process_image(img):
  # cv2.imshow('Original', img)
  height, width, ch = img.shape
  # print(height, width, ch)

  # Removing the center part of the original image
  new_height = int(height / 5) * 2
  new_width = int(width/2)
  start_width = int(width/4)
  cropped_img = np.zeros((new_height, new_width, 3), dtype=np.uint8)
  cropped_img[0:new_height,0:start_width,:] =         img[int(height / 5) * 3 :height,0:start_width,:]  
  cropped_img[0:new_height,start_width:new_width,:] = img[int(height / 5) * 3 :height,start_width + new_width: width,:]

  # Warp perspective
  src = np.float32([[0, 0], [new_width, 0], [new_width, new_height], [0, new_height]])
  dst = np.float32([[0, 0], [new_width, 0], [new_width, height], [0, height]])
  M = cv2.getPerspectiveTransform(src, dst)
  warped_img = cv2.warpPerspective(cropped_img, M, (new_width, height))
  # cv2.imshow('warped', warped_img)

  # yellow img, BGR
  yellow_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
  lower_yellow = np.array([0, 100, 100], dtype = "uint8")
  higher_yellow = np.array([100, 255, 255], dtype = "uint8")
  yellow_img = cv2.inRange(yellow_hsv, lower_yellow, higher_yellow)
  # cv2.imshow('yellow', yellow_img)

  # line detection
  minVal = 250
  maxVal = 400
  edges = cv2.Canny(yellow_img, minVal, maxVal)
  edges_copy = np.copy(edges)
  threshold = 10
  minLineLength= 30
  maxLineGap= 10
  lines = cv2.HoughLinesP(
      edges_copy,
      rho=1,
      theta=np.pi / 180,
      threshold=threshold,
      lines=np.array([]),
      minLineLength=minLineLength,
      maxLineGap=maxLineGap
      )
  
  line_img = np.zeros((yellow_img.shape[0],yellow_img.shape[1],3),dtype=np.uint8)
  line_color=[0, 255, 0]
  line_thickness=1
  dot_color = [0, 255, 0]
  dot_size = 3

  left_x_list = []
  left_y_list = []

  right_x_list = []
  right_y_list = []

  if lines is not None: 
      for line in lines:
          for x1, y1, x2, y2 in line:
              if x2 - x1 != 0:
                  slope = (y1 - y2) / (x1 - x2)
                  if 0.3 < abs(slope) < 10:
                      cv2.line(line_img, (x1, y1), (x2, y2), line_color,
                          line_thickness)
                      cv2.circle(line_img, (x1, y1), dot_size, dot_color, -1)
                      cv2.circle(line_img, (x2, y2), dot_size, dot_color, -1)

                      if slope < 0:
                          left_x_list.extend([x1, x2])
                          left_y_list.extend([y1, y2])
                      else:
                          right_x_list.extend([x1, x2])
                          right_y_list.extend([y1, y2])

  # settings of best fit line
  line_color=[255, 255, 255]
  line_thickness=2

  left_min_y = 0
  left_max_y = 0 
  left_min_x = 0
  left_max_x = 0
  if left_x_list and left_y_list:
      left_fit = np.polyfit(left_y_list, left_x_list, deg=1)
      left_poly = np.poly1d(left_fit)

      left_min_y = min(left_y_list)
      left_max_y = max(left_y_list)

      left_min_x = int(left_poly(left_min_y))
      left_max_x = int(left_poly(left_max_y))

      cv2.line(line_img, (left_min_x, left_min_y), (left_max_x, left_max_y), line_color,
          line_thickness)
  
  right_min_y = 0
  right_max_y = 0
  right_min_x = 0
  right_max_x = 0
  if right_x_list and right_y_list:
      right_fit = np.polyfit(right_y_list, right_x_list, deg=1)
      right_poly = np.poly1d(right_fit)

      right_min_y = min(right_y_list)
      right_max_y = max(right_y_list)

      right_min_x = int(right_poly(right_min_y))
      right_max_x = int(right_poly(right_max_y))

      cv2.line(line_img, (right_min_x, right_min_y ), (right_max_x, right_max_y), line_color,
              line_thickness)

  # car position
  # settings of car
  line_color=[255, 0, 0]
  line_thickness=2
  cv2.line(line_img, (int(new_width / 2), 0), (int(new_width / 2), height), line_color,
              line_thickness)

  # center 
  # settings of center
  line_color=[0, 255, 255]
  line_thickness=2

  cte_y_min = 0
  cte_y_max = 0
  if left_x_list and left_y_list and right_x_list and right_y_list:
      # min of the 2 maxes, so the height will be the same
      cte_y_max = min([left_max_y, right_max_y])
      cte_y_min = max([left_min_y, right_min_y])
      cv2.line(line_img, (int((left_min_x + right_min_x) / 2), cte_y_min), (int((left_max_x + right_max_x) / 2), cte_y_max), line_color,
                  line_thickness)
  elif left_x_list and left_y_list:
      cte_y_max = left_max_y
      cte_y_min = left_min_y
      cv2.line(line_img, (int((left_min_x + new_width) / 2), cte_y_min), (int((left_max_x + new_width) / 2), cte_y_max), line_color,
                  line_thickness)
  elif right_x_list and right_y_list:
      cte_y_max = right_max_y
      cte_y_min = right_min_y
      cv2.line(line_img, (int((right_min_x) / 2), cte_y_min), (int((right_max_x) / 2), cte_y_max), line_color,
                  line_thickness)
  else:
      print("NO LINES NO LINES NO LINES")

  # CTE logic
  cte = 0

  line_color=[0, 0, 255]
  line_thickness=2
  dot_color = [0, 0, 255]
  dot_size = 3
  if left_x_list and left_y_list and right_x_list and right_y_list:
      print('straight')
      cte = (new_width / 2) - ((left_max_x + right_max_x) / 2)

      first_x = int(new_width / 2)
      first_y = cte_y_max
      second_x = int((left_max_x + right_max_x) / 2)
      second_y = cte_y_max

      cv2.line(line_img, (first_x, first_y), (second_x, second_y), line_color,
                          line_thickness)
      cv2.circle(line_img, (first_x, first_y), dot_size, dot_color, -1)
      cv2.circle(line_img, (second_x, second_y), dot_size, dot_color, -1)
  elif left_x_list and left_y_list:
      print('right')
      cte_min = (new_width / 2) - ((left_min_x + new_width) / 2)
      cte_max = (new_width / 2) - ((left_max_x + new_width) / 2)
      cte = (cte_min + cte_max) / 2

      first_x = int(new_width / 2)
      first_y = cte_y_max
      second_x = int(((left_min_x + new_width) / 2) + ((left_max_x + new_width) / 2) / 2)
      second_y = cte_y_max

      cv2.line(line_img, (first_x, first_y), (second_x, second_y), line_color,
                          line_thickness)
      cv2.circle(line_img, (first_x, first_y), dot_size, dot_color, -1)
      cv2.circle(line_img, (second_x, second_y), dot_size, dot_color, -1)
  elif right_x_list and right_y_list:
      print('left')
      cte_min = (new_width / 2) - ((right_min_x) / 2)
      cte_max = (new_width / 2) - ((right_max_x) / 2)
      cte = (cte_min + cte_max) / 2

      first_x = int(new_width / 2)
      first_y = cte_y_max
      second_x = int(((right_min_x + new_width) / 2) + ((right_max_x + new_width) / 2) / 2)
      second_y = cte_y_max

      cv2.line(line_img, (first_x, first_y), (second_x, second_y), line_color,
                          line_thickness)
      cv2.circle(line_img, (first_x, first_y), dot_size, dot_color, -1)
      cv2.circle(line_img, (second_x, second_y), dot_size, dot_color, -1)

  # cv2.imshow('lines', line_img)

  # overlays lane lines w/ mask over color img
  overlay = cv2.addWeighted(warped_img, 1.0, line_img, 1.0, 0.0)
  cv2.imshow('overlay', overlay)
  cv2.waitKey(1) 

  return cte

# PID control to adjust car on center of road
def pid_control(recent_cte, elasped_time):
    global last_cte

    p_gain = - 0.27 * recent_cte
    
    d_gain = - 0.008 * (recent_cte - last_cte) / elasped_time
        
    pid = p_gain + d_gain

    print('p_gain: %f, d_gain: %f, pid: %f' % (p_gain, d_gain, pid))

    # if pid > 0:
    #     print('right')
    # elif pid < 0:
    #     print('left')
    # else: 
    #     print('straight')

    last_cte = recent_cte

    return pid


def main():

    rclpy.init()
    node = rclpy.create_node('lane_follower')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    steering_pub = node.create_publisher(Int64, 'cv_steer', 10)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    
    cred = credentials.Certificate("self-driving-e5e2d-firebase-adminsdk-zr5ow-fba56a97e1.json")
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    data = {}

    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok():
        start_time = time.time()
        # Process one image. The return value will be use for `something` later.
        c_cte = process_image(image)
        steer = pid_control(c_cte, time.time() - start_time)

        # Store the CTE
        data["CTE"] = int(c_cte)
        doc = db.collection("Lane Control").document()
        doc.set(data)

        steer_msg = Int64()
        steer_msg.data = int(steer)
        steering_pub.publish(steer_msg)
        # print("process time: %f" % (time.time() - start_time))
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()