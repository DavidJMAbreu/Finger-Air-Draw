import cv2 as cv
import numpy
from collections import deque


# Project Variables
color_configured = 0
skinLower = numpy.array([255, 255, 255])
skinHigher = numpy.array([0, 255, 255])


# Drawing variables
blue_points = [deque(maxlen=512)]
green_points = [deque(maxlen=512)]
red_points = [deque(maxlen=512)]
black_points = [deque(maxlen=512)]
yellow_points = [deque(maxlen=512)]
# Color indexes for drawings
blindex = 0
bindex = 0
gindex = 0
rindex = 0
yindex = 0
# Color array (BGR)
# black - blue - green - red - yellow
colors = [(0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)]
current_color = 0
# Variable to differentiate drawings
startDrawing = 0
stopDrawing = 0

# Drawing canvas initial setup
Canvas = numpy.zeros((480, 320, 3))+255
Canvas = cv.rectangle(Canvas, (10, 1), (52, 50), (0, 0, 0), 2)
Canvas = cv.rectangle(Canvas, (62, 1), (104, 50), colors[0], -1)
Canvas = cv.rectangle(Canvas, (114, 1), (156, 50), colors[1], -1)
Canvas = cv.rectangle(Canvas, (166, 1), (208, 50), colors[2], -1)
Canvas = cv.rectangle(Canvas, (218, 1), (260, 50), colors[3], -1)
Canvas = cv.rectangle(Canvas, (270, 1), (312, 50), colors[4], -1)
cv.putText(Canvas, "CL", (20, 20), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (0, 0, 0), 2, cv.LINE_AA)
cv.putText(Canvas, "0", (25, 40), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (0, 0, 0), 2, cv.LINE_AA)
cv.putText(Canvas, "1", (77, 30), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(Canvas, "2", (129, 30), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(Canvas, "3", (181, 30), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(Canvas, "4", (233, 30), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(Canvas, "5", (285, 30), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(Canvas, "Drawing", (110, 460), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (0, 0, 0), 2, cv.LINE_AA)
cv.putText(Canvas, "OFF", (180, 460), cv.FONT_HERSHEY_SIMPLEX,
           0.5, (0, 0, 255), 2, cv.LINE_AA)


# Draw the circles in the fram to run the calibration of the skin color
def draw_hand_circle(drawing_frame):
    rows, cols, _ = drawing_frame.shape
    global number_circle, circle_y, circle_x

    # y position of the circle set
    circle_y = numpy.array(
        [6 * rows / 20, 6 * rows / 20, 6 * rows / 20,
         9 * rows / 20, 9 * rows / 20, 9 * rows / 20,
         12 * rows / 20, 12 * rows / 20, 12 * rows / 20],
        dtype=numpy.uint32)

    # x position of the circle set
    circle_x = numpy.array(
        [15 * cols / 20, 16.5 * cols / 20, 18 * cols / 20,
         15 * cols / 20, 16.5 * cols / 20, 18 * cols / 20,
         15 * cols / 20, 16.5 * cols / 20, 18 * cols / 20],
        dtype=numpy.uint32)

    # Total number of circles (9)
    number_circle = len(circle_y)

    # Draw the circles on the frame
    for i in range(number_circle):
        cv.circle(drawing_frame,
                  (circle_x[i], circle_y[i]), 10, (0, 0, 255), 1)

    # Help text
    cv.putText(drawing_frame, "Position hand and press 's'", (200, 30),
               cv.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 0), 1)


# Function to calibrate the skin color (HSV based segmentation)
def color_calibration(calibration_frame):
    global skinHigher, skinLower, color_configured

    # Convert frame to HSV
    calibration_frame = cv.cvtColor(calibration_frame, cv.COLOR_BGR2HSV)

    # Variables to store max and min values
    l_v = 255
    l_h = 255
    l_s = 255
    g_v = 0
    g_h = 0
    g_s = 0

    for i in range(number_circle):

        # Get the mean value in a 9x9 square in each circle
        v = 0
        s = 0
        h = 0
        for j in range(9):
            for k in range(9):
                h += calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)][0]
                s += calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)][1]
                v += calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)][2]

        v /= 81
        s /= 81
        h /= 81

        # Save the max
        if(v > g_v and s > g_s and h > g_h):
            g_v = round(v)
            g_s = round(s)
            g_h = round(h)

        # Save the min
        if(v < l_v and s < l_s and h < l_h):
            l_v = round(v)
            l_s = round(s)
            l_h = round(h)

    # Save the skin colors
    skinHigher = numpy.array([round(g_h)+50, round(g_s)+100, 255])
    skinLower = numpy.array([round(l_h)-20, round(l_s)-30, round(l_v)-80])

    color_configured = 1


# Function to get the cursor position on the index finger
# Furthest point based algorithm
def getPoint(contour):
    lines = []
    points = []
    index = 0
    # Get highest line on the contour
    for c in contour:
        lines.append(c[0][1])

    highest = min(lines)
    while(index < len(lines)):
        try:
            new_find = lines.index(highest, index)
            points.append(new_find)
            index = new_find+1
        except ValueError as e:
            break

    # Get every column in the line and get the mean of those pixels
    col = 0
    for point in points:
        col += contour[point][0][0]
    col = round(col/len(points))

    return [highest, col]


# Function to save the current drawing pixel in the correct array
def savePoint(point):
    if current_color == 0:
        black_points[blindex].appendleft(point)
    elif current_color == 1:
        blue_points[bindex].appendleft(point)
    elif current_color == 2:
        green_points[gindex].appendleft(point)
    elif current_color == 3:
        red_points[rindex].appendleft(point)
    elif current_color == 4:
        yellow_points[yindex].appendleft(point)


# Function to clear the Canvas
def clean(Canvas, drawing='off'):
    Canvas = numpy.zeros((480, 320, 3))+255
    Canvas = cv.rectangle(Canvas, (10, 1), (52, 50), (0, 0, 0), 2)
    Canvas = cv.rectangle(Canvas, (62, 1), (104, 50), colors[0], -1)
    Canvas = cv.rectangle(
        Canvas, (114, 1), (156, 50), colors[1], -1)
    Canvas = cv.rectangle(
        Canvas, (166, 1), (208, 50), colors[2], -1)
    Canvas = cv.rectangle(
        Canvas, (218, 1), (260, 50), colors[3], -1)
    Canvas = cv.rectangle(
        Canvas, (270, 1), (312, 50), colors[4], -1)
    cv.putText(
        Canvas, "CL", (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv.LINE_AA)
    cv.putText(Canvas, "0", (25, 40), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (0, 0, 0), 2, cv.LINE_AA)
    cv.putText(Canvas, "1", (77, 30), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (255, 255, 255), 2, cv.LINE_AA)
    cv.putText(Canvas, "2", (129, 30), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (255, 255, 255), 2, cv.LINE_AA)
    cv.putText(Canvas, "3", (181, 30), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (255, 255, 255), 2, cv.LINE_AA)
    cv.putText(Canvas, "4", (233, 30), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (255, 255, 255), 2, cv.LINE_AA)
    cv.putText(Canvas, "5", (285, 30), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (255, 255, 255), 2, cv.LINE_AA)
    cv.putText(Canvas, "Drawing", (110, 460), cv.FONT_HERSHEY_SIMPLEX,
               0.5, (0, 0, 0), 2, cv.LINE_AA)
    if drawing == 'off':
        cv.putText(Canvas, "OFF", (180, 460), cv.FONT_HERSHEY_SIMPLEX,
                   0.5, (0, 0, 255), 2, cv.LINE_AA)
    else:
        cv.putText(Canvas, "ON", (180, 460), cv.FONT_HERSHEY_SIMPLEX,
                   0.5, (0, 255, 0), 2, cv.LINE_AA)
    return Canvas

# Draw the points arrays in the drawing canvas


def draw(Canvas, isDrawing):

    Canvas = clean(Canvas, isDrawing)

    points = [black_points, blue_points,
              green_points, red_points, yellow_points]
    for i in range(len(points)):
        for j in range(len(points[i])):
            for k in range(1, len(points[i][j])):
                if points[i][j][k - 1] is None or points[i][j][k] is None:
                    continue
                Canvas = cv.line(Canvas, points[i][j][k - 1],
                                 points[i][j][k], colors[i], 2)

    return Canvas


# Load the video from first webcam in list
webcam = cv.VideoCapture(0)


if not webcam.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
while True:

    ret, frame = webcam.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    # Invert the video
    frame = cv.flip(frame, 1)

    # Save a secondary frame to present the image
    frame_circle = frame

    if color_configured == 1:
        # Check only a partial part of the frame (half of it)
        [lin, col, pln] = numpy.shape(frame)
        hand_frame = frame[0:480, round(col/2):col, 0:3]

        # Convert to hsv
        mask = cv.cvtColor(hand_frame, cv.COLOR_BGR2HSV)
        # Perform the skin color segmentation
        mask = cv.inRange(mask, skinLower, skinHigher)
        cv.imshow("Mask", mask)

        # Morphology operations to enhance the algorithm
        mask = cv.morphologyEx(mask, cv.MORPH_DILATE, numpy.ones(
            (3, 3), dtype=numpy.uint8), iterations=4)
        mask = cv.morphologyEx(mask, cv.MORPH_ERODE, numpy.ones(
            (3, 3), dtype=numpy.uint8), iterations=2)
        cv.imshow("Mask after open", mask)

        # Perform a median filter on the mask
        medianFilter = cv.medianBlur(mask, 11)
        medianFilter = cv.medianBlur(medianFilter, 11)
        medianFilter = cv.medianBlur(medianFilter, 11)
        cv.imshow("median", medianFilter)

        # Find the contours
        contours, hierarchy = cv.findContours(
            medianFilter, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours != []:
            # Detect the largest contour
            largest = 0
            max_area = 0
            for contour in contours:
                area = cv.contourArea(contour)
                if (area > max_area):
                    max_area = area
                    largest = contour

            # Get cursor position
            column, line = getPoint(largest)
            cv.drawContours(frame_circle[0:480, round(
                col/2):col, 0:3], largest, -1, (0, 255, 255), 2)

            # Draw cursor on frame and drawing canvas for user orientation
            cv.circle(frame_circle[0:480, round(col/2):col, 0:3],
                      (line, column+15), 2, colors[current_color], -1)
            Canvas = clean(Canvas, 'on' if startDrawing == 1 else 'off')

            if key != -1:
                if key == ord('d') and startDrawing == 0:  # Draw
                    stopDrawing = 0
                    startDrawing = 1
                elif key == ord('s') and startDrawing == 1:  # Stop drawing
                    stopDrawing = 1
                    startDrawing = 0

            if startDrawing == 1:  # Save the new pixel
                savePoint([line, column+15])
            elif stopDrawing == 1:  # Create new deque's to separate different draws
                stopDrawing = 0
                black_points.append(deque(maxlen=512))
                blindex += 1
                blue_points.append(deque(maxlen=512))
                bindex += 1
                red_points.append(deque(maxlen=512))
                rindex += 1
                green_points.append(deque(maxlen=512))
                gindex += 1
                yellow_points.append(deque(maxlen=512))
                yindex += 1

            # Draw on the Canvas
            Canvas = draw(Canvas, 'on' if startDrawing == 1 else 'off')
            Canvas = cv.circle(Canvas, (line, column+15),
                               2, colors[current_color], -1)

    # Color calibration if "s" is pressed
    if color_configured == 0 and cv.waitKey(1) == ord("s"):
        color_calibration(frame)

    if color_configured == 1:
        if cv.waitKey(1) == ord("r"):  # Reset and restart calibration
            color_configured = 0
        else:
            key = cv.waitKey(1)
            if key != -1:
                if key == ord("0"):  # Clear drawing canvas
                    black_points = [deque(maxlen=512)]
                    blue_points = [deque(maxlen=512)]
                    green_points = [deque(maxlen=512)]
                    red_points = [deque(maxlen=512)]
                    yellow_points = [deque(maxlen=512)]
                    blindex = 0
                    bindex = 0
                    gindex = 0
                    rindex = 0
                    yindex = 0

                    startDrawing = 0
                    stopDrawing = 0
                    Canvas = clean(Canvas, 'off')
                else:
                    if key == ord("1"):  # Draw black
                        current_color = 0
                    elif key == ord("2"):  # Draw blue
                        current_color = 1
                    elif key == ord("3"):  # Draw green
                        current_color = 2
                    elif key == ord("4"):  # Draw red
                        current_color = 3
                    elif key == ord("5"):  # Draw yellow
                        current_color = 4

    if color_configured == 0:
        draw_hand_circle(frame_circle)  # Draw the circles on frame

    # Concat frame and canvas (cv.hconcat doesn't work on images with different sizes)
    # Get image sizes
    frame_lin, frame_col, _ = numpy.shape(frame_circle)
    _, Canvas_col, _ = numpy.shape(Canvas)
    total_width = frame_col+Canvas_col
    total_height = frame_lin

    # Create output matrix
    canvas = numpy.ones((total_height, total_width, 3), dtype=numpy.uint8)

    # Concat images
    canvas[0:total_height, 0:frame_col, :] = frame_circle[:, :, 0:3]
    canvas[0:total_height, frame_col:total_width, :] = Canvas[:, :, 0:3]

    cv.imshow("Air Canvas", canvas)  # Show the output

    # Key 27 == escape key
    if cv.waitKey(1) == 27:
        break
