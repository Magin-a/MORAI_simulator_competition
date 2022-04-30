#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  #np.fromstring으로 바이트를 uint8 array로 변환
        except CvBridgeError as e: #다시 세로, 가로, 채널 의 이미지를 array로 변환
            print(e)

        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1) #토픽에서 받은 이미지를 띄운다.


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 