#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

TRUNCATE_BOTTOM = 0.3

bridge = CvBridge()

image_pub = rospy.Publisher('/image_feed', Image, queue_size=1)


def process_pub_img(cv_image):

    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    sat_frame = hsv_frame[:,:,1]
    ret, bw_frame = cv2.threshold(sat_frame, 75, 255, cv2.THRESH_BINARY_INV)

    truncate_start = round((1 - TRUNCATE_BOTTOM) * (len(cv_image[0]) - 1))
    bw_frame_truncated = bw_frame[truncate_start:, :]

    image_pub.publish(bridge.cv2_to_imgmsg(bw_frame_truncated, encoding="passthrough"))


def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)

    process_pub_img(cv_image)
    

rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/rrbot/camera1/image_raw', Image, callback)

rospy.spin()
