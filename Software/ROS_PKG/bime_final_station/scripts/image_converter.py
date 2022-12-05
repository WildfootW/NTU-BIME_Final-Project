#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Copy from https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# Modified for Noetic
#
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_cv", Image, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Return cv::Mat
        except CvBridgeError as e:
            rospy.logerr(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        # Display the Image Directly
        cv2.imshow("ImageConverter cv2 display", cv_image) 
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

def image_converter():
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()
    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    image_converter()
