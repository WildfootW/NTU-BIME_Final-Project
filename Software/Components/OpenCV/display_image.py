import cv2
import sys
img = cv2.imread(cv2.samples.findFile("../HC-SR04/HC-SR04-1.1.jpg")) # After reading in the image data will be stored in a cv::Mat object.
if img is None:
    sys.exit("Could not read the image.")
cv2.imshow("Display window", img) # The first argument is the title of the window
k = cv2.waitKey(0) # Zero means to wait forever. The return value is the key that was pressed.
if k == ord("s"):
    cv2.imwrite("new_file.png", img)
