# Required moduls
import cv2
import numpy

# Constants for finding range of skin color in YCrCb
min_YCrCb = numpy.array([0,133,77],numpy.uint8)
max_YCrCb = numpy.array([255,173,127],numpy.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX
gameflag = False

# Create a window to display the camera feed
cv2.namedWindow('Camera Output')

# Get pointer to video frames from primary device
videoFrame = cv2.VideoCapture(0)

while True: # any key pressed has a value >= 0

    # Grab video frame, decode it and return next video frame
    readSucsess, sourceImage = videoFrame.read()
    sourceImage = cv2.flip(sourceImage, 1)

    # Convert image to YCrCb
    imageYCrCb = cv2.cvtColor(sourceImage,cv2.COLOR_BGR2YCR_CB)

    # Find region with skin tone in YCrCb image
    skinRegion = cv2.inRange(imageYCrCb,min_YCrCb,max_YCrCb)
    
    skin = cv2.bitwise_and(sourceImage, sourceImage, mask = skinRegion)

    # Do contour detection on skin region
    contours, hierarchy = cv2.findContours(skinRegion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.rectangle(skin,(0,300),(640,200),(0,0,255),3)
    if len(contours) != 0:
        contour = max(contours, key = cv2.contourArea)
        M = cv2.moments(contour)
        if(M["m00"] != 0):
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            area = M["m00"]
            if cy < 200:
                cv2.circle(skin, (cx, cy), 7, (0, 255, 0), -1)
            elif cy > 300:
                cv2.circle(skin, (cx, cy), 7, (255, 0, 0), -1)
            else:
                cv2.circle(skin, (cx, cy), 7, (255, 255, 255), -1)
                gameflag = True
                

    # Display the source image
    cv2.imshow('Camera Output',skin)

    # Check for user input to close program
    if cv2.waitKey(5) == 27: # wait 5 millisecond in each iteration of while loop
        break 

# Close window and camera after exiting the while loop
cv2.destroyWindow('Camera Output')
videoFrame.release()