#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022  <>
#
# Merge Color Detect and Image Converter
# 
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def detect_color(imageFrame):
  
    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
  
    # Set range for red color and 
    # define mask
    red_lower = np.array([160, 20, 70], np.uint8)
    red_upper = np.array([190, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2) 
    # Set range for green color and 
    # define mask
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
    green_mask = cv2.erode(green_mask, None, iterations=2)
    green_mask = cv2.dilate(green_mask, None, iterations=2) 

    # Set range for blue color and
    # define mask
    blue_lower = np.array([101, 50, 38], np.uint8)
    blue_upper = np.array([110, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2) 
    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")
      
    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
      
    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = green_mask)
      
    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask = blue_mask)
   
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 50000):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h), 
                                       (0, 0, 255), 2)
              
            cv2.putText(imageFrame, "Red Colour, Target :Green ", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))    
            

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 50000):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
              
            cv2.putText(imageFrame, "Green Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 255, 0))
  
    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 50000):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "Blue Colour, Target: Yellow", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))

    return imageFrame

def text_information(imageFrame):
    DHT11 = 100
    left_pedding = 12
    right_pedding = 12
    cv2.putText(imageFrame ,"Temperature: " + str(DHT11) , (100,50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 255, 255),5)
    cv2.putText(imageFrame ,"L Distance: " + str(left_pedding) , (100,100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 255, 255),5)
    cv2.putText(imageFrame ,"R Distance: " + str(right_pedding) , (100,150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 255, 255),5)
    return imageFrame


cv_bridge_instance = CvBridge()
#@static_vars(cv_bridge_instance = CvBridgeError())
def image_to_cv2(data):
    try:
        cv_image = cv_bridge_instance.imgmsg_to_cv2(data, "bgr8") # Return cv::Mat
    except CvBridgeError as e:
        rospy.logerr(e)

    (rows, cols, channels) = cv_image.shape
    if cols > 60 and rows > 60 :
        cv2.circle(cv_image, (50,50), 10, 255)

    # Display the Image Directly
    #cv2.imshow("ImageConverter cv2 display", cv_image) 
    #cv2.waitKey(3)

    return cv_image

def callback(frame):
    frame = image_to_cv2(frame)
    frame = detect_color(frame)
    frame = text_information(frame)

    cv2.imshow("On Screen Display", frame)
    cv2.waitKey(3)

def on_screen_display():
    rospy.init_node('on_screen_display', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    rospy.spin()
    rospy.loginfo(rospy.get_caller_id() + " Spin END")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    on_screen_display()
