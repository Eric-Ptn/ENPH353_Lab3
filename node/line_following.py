#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

FORWARD_SPEED = 1
ANGULAR_PROPORTIONALITY_CONSTANT = 0.02
TRUNCATE_BOTTOM = 0.3

bridge = CvBridge()


velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
# debug_pub = rospy.Publisher('/debug', String, queue_size=10)
# image_pub = rospy.Publisher('/image_feed', Image, queue_size=1)

def find_road(cv_image):
    global prev_cX

    if 'prev_cX' not in globals():
        prev_cX = 0

    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    sat_frame = hsv_frame[:,:,1]
    _, bw_frame = cv2.threshold(sat_frame, 75, 255, cv2.THRESH_BINARY)

    truncate_start = round((1 - TRUNCATE_BOTTOM) * (len(cv_image[0]) - 1))
    bw_frame_truncated = bw_frame[truncate_start:, :]

    M = cv2.moments(bw_frame_truncated)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
    else:
        cX = prev_cX # if no road was found, send the move function the old centroid location

    prev_cX = cX

    # cv2.circle(cv_image, (cX, 50), 10, (255, 255, 255), -1)

    # image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    # image_pub.publish(bridge.cv2_to_imgmsg(bw_frame_truncated, encoding='passthrough'))


    return cX


def move_bot(cv_image):
    move = Twist()
    move.linear.x = FORWARD_SPEED

    road_centroid = find_road(cv_image)

    move.angular.z = ANGULAR_PROPORTIONALITY_CONSTANT * ((np.shape(cv_image)[1])/2 - road_centroid)

    velocity_pub.publish(move)


def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)

    move_bot(cv_image)    

rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/rrbot/camera1/image_raw', Image, callback)

rospy.spin()
