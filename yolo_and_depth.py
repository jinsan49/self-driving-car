#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time
import numpy as np
from std_msgs.msg import Int64
from ultralytics import YOLO
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = "/zed/zed_node/stereo/image_rect_color"
#Trained Model
MODEL_FILE_NAME = "yolo_model/runs/detect/tf_yolo_custom7/weights/best.pt"
print('Loading %s' % MODEL_FILE_NAME)
model = YOLO(MODEL_FILE_NAME)

br = CvBridge()
image = None
dimg = None

# LiDAR data
scan_data = None
def scan_callback(data):
    global scan_data
    scan_data = data

# ZED Camera info
def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]

# Find out the images by custom YOLO and add classification on cropped image.
def yolo_image(img):
    # cv2.imshow('Original', img)
    height, width, ch = img.shape
    # get the left img from LR img
    new_height = height
    new_width = int(width/2)
    cropped_img = np.zeros((new_height, new_width, 3), dtype=np.uint8)
    cropped_img[:,0:new_width,:] = img[:,0:new_width,:]  

    # send left img as input to model
    results = model(cropped_img, verbose=False)

    # prediction (0: green, 1: Red, 3: Yellow)
    class_label = 0
    for r in results:
        for box in r.boxes:
            #getting coordinate of bounding boxes
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            # Draw bounding box on the img 
            cv2.rectangle(cropped_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            class_label = int(box.cls.item())
            confidence_score = box.conf.item()

            label =f"Class: {class_label}, Confidence: {confidence_score:.2f}" 
            cv2.putText(cropped_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('overlay', cropped_img )
    cv2.waitKey(1)

    return class_label

# Depth Detection that finding the closest distance inside of the range (45 degree left/right from front)
def depth_range():
    range_array = np.array(scan_data.ranges, copy=True)
    front_distances = (np.append(range_array[-5:], range_array[:6]) * 100).astype(int)

    return np.min(front_distances)

def main():

    rclpy.init()
    node = rclpy.create_node('tf_yolo')
    node = rclpy.create_node('lidar_view')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 1000)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    throttle_pub = node.create_publisher(Int64, 'yolo_throttle', 10)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    
    throttle_msg = Int64()
    throttle_msg.data = 16
    throttle_pub.publish(throttle_msg)
    
    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok() and scan_data is None:
        print("Not receiving lidar data")
        rate.sleep()

    green_wait = False

    cred = credentials.Certificate("self-driving-e5e2d-firebase-adminsdk-zr5ow-fba56a97e1.json")
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    data = {}
    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        light = yolo_image(image)
        distance = depth_range()
        
        if light == 0: # green
            throttle = 17
            green_wait = False
        elif light == 1 or green_wait: #red
            throttle = 0
            green_wait = True
        # elif light == 3 :# yellow
        #     throttle = 15  
        else:   # if you can't see a light, its green
            throttle = 17

        # Throttle update to Cloud database 
        data["Throttle"] = int(throttle)
        # print values, no detection means green too
        light_val = {0: 'Green', 1: 'Red', 3: 'Yellow'}

        print('light: %s, closest_dist: %d cm, throttle: %d' % (light_val[light], distance, throttle))

        # stop when it detects object within 70 cm
        if distance < 70:
            throttle = 0
        # LiDAR distance update to Cloud database 
        data["distance"] = int(distance)
        doc = db.collection("yolo and depth").document()
        doc.set(data)

        throttle_msg.data = throttle
        throttle_pub.publish(throttle_msg)

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()