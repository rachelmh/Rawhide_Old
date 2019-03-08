# import the necessary packages
import cv2
import numpy as np
# #Declare variables for window
# cv2.namedWindow("preview")
# vc = cv2.VideoCapture(2)
# # try to get the first frame
# if vc.isOpened():
#     rval, frame = vc.read()
# else:
#     rval = False
# while rval:
#     cv2.imshow("preview", frame)
#     rval, frame = vc.read()
#     key = cv2.waitKey(20)
#     if key == 27: # Exit when ESC key is pressed
#         break
# vc.release() #To unlock camera on windows OS
# cv2.destroyWindow("preview")




#Declare variables for window
vc = cv2.VideoCapture(2)
while(1):
    # Take each frame
    ret, frame = vc.read()
    if(ret):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_blue = np.array([50, 100, 100], dtype=np.uint8)#[110,50,50]
        upper_blue = np.array([70,255,255], dtype=np.uint8)#[130,255,255]
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        k = cv2.waitKey(5) & 0xFF
    if k == 27: #Exit when ESC key is pressed
        break
vc.release()
cv2.destroyAllWindows()