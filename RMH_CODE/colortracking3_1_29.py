import cv2
import numpy as np

#Limegreen  H=[47 57], S=[57 255] V=[0 255]
#teal  H=
usePreset = False       # Set this as 'True' to use preset HSV values 


cap = cv2.VideoCapture(2)
# Set the video width and height
cap.set(3, 640)
cap.set(4, 480)

# Creating the labels and min max values for the trackbars
hsv_label = ['H_min', 'H_max', 'S_min', 'S_max', 'V_min', 'V_max']
hsv_value = [0, 179, 0, 255, 0, 255]

prDn = False

# Defining kernals to smooth the image
kernel_erode = np.ones([3,3], np.uint8)
kernel_dilate = np.ones([8,8], np.uint8)

# Trackbar call back function
def nothing(x):
    pass

def presetHSV(req):
    global prDn
    # Save your HSV colour values in 'colour_value' and label in 'color'
    # [H_min, H_max, S_min, S_max, V_min, V_max]
    color = ['green', 'blue', 'orange']
    color_value = [[50, 78, 85, 154, 107, 145],     # green
                   [104, 137, 103, 226, 50, 168],   # blue
                   [0, 59, 3, 131, 214, 255]]   # orange
    for i in color:
        v = color.index(i)
        if i == str(req):
            for u in range(6):
                hsv_value[u] = color_value[v][u]
    prDn = True
    
def getHSV():
    # Read the trackbars to gen the HSV values
    for i in range(6):
        hsv_value [i] = cv2.getTrackbarPos(hsv_label[i], 'HSV_Select')
        
def createTrackbars():    
    #['H_min', 'H_max', 'S_min', 'S_max', 'V_min', 'V_max']
    for i in range(6):
        if i % 2 == 0:
            cv2.createTrackbar(hsv_label[i], 'HSV_Select', hsv_value[i], hsv_value[i+1], nothing)
        if i % 2 == 1:
            cv2.createTrackbar(hsv_label[i], 'HSV_Select', hsv_value[i], hsv_value[i], nothing)
    
if usePreset == False:
    cv2.namedWindow('HSV_Select', 1)
    cv2.resizeWindow('HSV_Select', 450, 310)
    createTrackbars()

while(1):
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    if usePreset == True and prDn == False:
        presetHSV('orange')        # Put preset color value
    elif usePreset == False:
        getHSV()
    
    max_hsv = np.array([hsv_value[1], hsv_value[3], hsv_value[5]])
    min_hsv = np.array([hsv_value[0], hsv_value[2], hsv_value[4]])
    
    mask = cv2.inRange(hsv_frame, min_hsv, max_hsv)
    
    # We only need to check frame and the mask so we set the first two inputs to frame
    res = cv2.bitwise_and(frame, frame, mask = mask)
    
    erode = cv2.erode(mask, kernel_erode)
    erode = cv2.erode(erode, kernel_erode)
    dilate = cv2.dilate(erode, kernel_dilate)
    dilate = cv2.dilate(dilate, kernel_dilate)
    
    _, contours, hierarchy = cv2.findContours(dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours
    for c in cnt:
        M = cv2.moments(c)
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M['m00'])
        
        
        cv2.drawContours(frame, [c], -1, (0, 255,255), 2)
        cv2.circle(frame, (x, y), 1, (0, 0, 0), -1)
        cv2.putText(frame, '(' + str(x) + ',' + str(y) + ')', (x + 20,  y + 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
    cv2.imshow('Capture', frame)
    cv2.imshow('res', res)
    #cv2.imshow('Mask', mask)
    #cv2.imshow('Erosion and Dilation', dilate)        # if you want to show this comment out contours before
    if cv2.waitKey(5) == 27:                       
        # Escape to brake the loop
        break
        
cap.release()
cv2.destroyAllWindows()
print ("Last run HSV values - ", hsv_value)