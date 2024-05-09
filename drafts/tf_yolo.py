#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import numpy as np
from std_msgs.msg import Int64
from ultralytics import YOLO

#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = "/zed/zed_node/stereo/image_rect_color"
#Trained Model
MODEL_FILE_NAME = "yolo_model/runs/detect/tf_yolo_custom7/weights/best.pt"
print('Loading %s' % MODEL_FILE_NAME)
model = YOLO(MODEL_FILE_NAME)

br = CvBridge()
image = None
dimg = None

def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]

def yolo_image(img):

    # cv2.imshow('Original', img)
    height, width, ch = img.shape
    # print(height, width, ch)

    # Removing the center part of the original image
    # cropped_img = img
    new_height = height
    new_width = int(width/2)
    cropped_img = np.zeros((new_height, new_width, 3), dtype=np.uint8)
    cropped_img[:,0:new_width,:] = img[:,0:new_width,:]  

    # overlays lane lines w/ mask over color img
    results = model(cropped_img)

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


def main():

    rclpy.init()
    node = rclpy.create_node('tf_yolo')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    throttle_pub = node.create_publisher(Int64, 'yolo_throttle', 10)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    
    throttle_msg = Int64()
    throttle_msg.data = 17
    throttle_pub.publish(throttle_msg)
    
    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        light = yolo_image(image)

        # ADD CODE FOR LIGHT 
        print(light)
        
        if light == 1 : #red
            throttle_msg.data = 0
        elif light == 3 :# yellow
            throttle_msg.data = 15  
        else:
            throttle_msg.data = 16
        throttle_pub.publish(throttle_msg)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()