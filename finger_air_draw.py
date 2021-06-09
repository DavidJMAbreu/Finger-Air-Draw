import cv2 as cv
import numpy
from numpy.lib.function_base import median


# Carregar o video da camêra principal
webcam = cv.VideoCapture(0)

# Variáveis de projeto
global color_configured
color_configured = 0
global skinLower, skinHigher
skinLower = numpy.array([255, 255, 255])
skinHigher = numpy.array([0, 255, 255])


# Define a 5x5 kernel for erosion and dilation
kernel = numpy.ones((5, 5), numpy.uint8)


# Função para desenhar os circulos de demonstração da área de aquisição de cor para o histograma
def draw_hand_circle(drawing_frame):
    rows, cols, _ = drawing_frame.shape
    global number_circle, circle_y, circle_x

    # Array da posição y de cada circulo
    circle_y = numpy.array(
        [6 * rows / 20, 6 * rows / 20, 6 * rows / 20,
         9 * rows / 20, 9 * rows / 20, 9 * rows / 20,
         12 * rows / 20, 12 * rows / 20, 12 * rows / 20],
        dtype=numpy.uint32)

    # Array da posição x de cada circulo
    circle_x = numpy.array(
        [15 * cols / 20, 16.5 * cols / 20, 18 * cols / 20,
         15 * cols / 20, 16.5 * cols / 20, 18 * cols / 20,
         15 * cols / 20, 16.5 * cols / 20, 18 * cols / 20],
        dtype=numpy.uint32)

    # Número total de circulos do vetor(9)
    number_circle = len(circle_y)

    # Desenhar os circulos na imagem
    for i in range(number_circle):
        cv.circle(drawing_frame,
                  (circle_x[i], circle_y[i]), 10, (0, 0, 255), 1)

    # Text de indicação
    cv.putText(drawing_frame, "Position hand and press 's'", (200, 30),
               cv.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 0), 1)


# Função para calibrar os vetores de cor a procurar na imagem (em HSV)
def color_calibration(calibration_frame):
    global skinHigher, skinLower, color_configured

    # Passar a calibration_frame para HSV para funcionarmos em relação de cor e intensidade
    calibration_frame = cv.cvtColor(calibration_frame, cv.COLOR_BGR2HSV)

    l_v = 255
    l_h = 255
    l_s = 255
    g_v = 0
    g_h = 0
    g_s = 0

    for i in range(number_circle):

        # Obter o valor médio numa vizinha de 9x9 pixeis
        v = 0
        s = 0
        h = 0
        for j in range(9):
            for k in range(9):
                print(calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)])
                h += calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)][0]
                s += calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)][1]
                v += calibration_frame[(circle_y[i]-4+j), (circle_x[i]-4+k)][2]

        v /= 81
        s /= 81
        h /= 81

        if(v > g_v and s > g_s and h > g_h):
            g_v = round(v)
            g_s = round(s)
            g_h = round(h)

        if(v < l_v and s < l_s and h < l_h):
            l_v = round(v)
            l_s = round(s)
            l_h = round(h)

    skinHigher = numpy.array([round(g_h)+50, round(g_s)+100, 255])
    skinLower = numpy.array([round(l_h)-20, round(l_s)-30, round(l_v)-80])
    print(skinLower)
    print("\n\n", skinHigher)

    color_configured = 1


def doNothing(x):
    pass


# Create a window
cv.namedWindow('Mask')

if not webcam.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
while True:
    ret, frame = webcam.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    # Inverter o video
    frame = cv.flip(frame, 1)

    frame_circle = frame

    if color_configured == 1:

        [lin,col,pln] = numpy.shape(frame)
        hand_frame = frame[0:480,round(col/2):col,0:3]
        mask = cv.cvtColor(hand_frame, cv.COLOR_BGR2HSV)

        mask = cv.inRange(mask, skinLower, skinHigher)

        mask = cv.morphologyEx(mask,cv.MORPH_DILATE,numpy.ones((3,3),dtype=numpy.uint8),iterations=4)
        mask = cv.morphologyEx(mask,cv.MORPH_ERODE,numpy.ones((3,3),dtype=numpy.uint8),iterations=2)

        medianFilter = cv.medianBlur(mask, 11)
        gaussian = cv.GaussianBlur(mask, (5, 5), 0)


        

        contours, hierarchy = cv.findContours(medianFilter, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
         
        cv.drawContours(medianFilter, contours, -1, 255, -1)
        if contours != []:
            # Detetar qual a maior àrea
            largest = 0
            max_area = 0
            for contour in contours:
                area = cv.contourArea(contour)
                if (area>max_area):
                    max_area = area
                    largest = contour

            centroid =  centroid(largest)
            #approx the contour a little
            epsilon = 0.0005*cv.arcLength(largest,True)
            approx= cv.approxPolyDP(largest,epsilon,True)

            hull= cv.convexHull(largest,returnPoints=True)
             #define area of hull and area of hand
            areahull = cv.contourArea(hull)
            areacnt = cv.contourArea(largest)

        #find the percentage of area not covered by hand in convex hull
            arearatio=((areahull-areacnt)/areacnt)*100

        #find the defects in convex hull with respect to hand
            hull = cv.convexHull(approx, returnPoints=False)
            defects = cv.convexityDefects(approx, hull)

            hull= cv.convexHull(largest,returnPoints=False)
            draw_hull= cv.convexHull(largest)
            cv.drawContours(frame[0:480,round(col/2):col,0:3], contours,0,(0,255,0),2)
            #Obter o ponto mais longe do centro
            for defect in defects:
                print('defeito ',largest[defect[0][2]][0])
                cv.circle(frame[0:480,round(col/2):col,0:3],largest[defect[0][2]][0],10,(255,255,255),3)
                
                    
                    
        
            

        cv.imshow("Mask", mask)
        cv.imshow("median", medianFilter)

    # Se durante o periodo de configuração a tecla Ctrl for pressionada
    if color_configured == 0 and cv.waitKey(10) == 115:
        # Fazer o histograma das cores da mão com base nos circulos definidos
        color_calibration(frame)

    if color_configured == 0:
        draw_hand_circle(frame_circle)

    cv.imshow("Video", frame_circle)

    if cv.waitKey(10) == 27:
        break
