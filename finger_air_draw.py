import cv2 as cv
import numpy


# Carregar o video da camêra principal
webcam = cv.VideoCapture(0)

# Variáveis de projeto
global color_configured
color_configured = 0
global skinLower, skinHigher
skinLower = numpy.array([255,255,255])
skinHigher = numpy.array([0,255,255])




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
        cv.circle(drawing_frame, (circle_x[i], circle_y[i]), 10, (0, 0, 255), 1)

    # Text de indicação
    cv.putText(drawing_frame, "Position hand and press 's'", (200, 30),
               cv.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 0), 1)



# Função para calibrar os vetores de cor a procurar na imagem (em HSV)
def color_calibration(calibration_frame):
    global skinHigher,skinLower,color_configured

    cv.imshow("Calibration",calibration_frame)

    # Passar a calibration_frame para HSV para funcionarmos em relação de cor e intensidade
    calibration_frame = cv.cvtColor(calibration_frame, cv.COLOR_BGR2HSV)

    current = numpy.array([],
                         dtype=numpy.uint8)

    for i in range(number_circle):
    
        # Obter o valor médio numa vizinha de 9x9 pixeis
        h=0
        s=0
        v=0
        for j in range(9):
            for k in range(9):
                h += calibration_frame[(circle_y[i]-4+j),(circle_x[i]-4+k)][0]
                s += calibration_frame[(circle_y[i]-4+j),(circle_x[i]-4+k)][1]
                v += calibration_frame[(circle_y[i]-4+j),(circle_x[i]-4+k)][2]
            
        h /= 81
        s /= 81
        v /= 81

        current = [round(h),round(s),round(v)]
        if (current[0]>skinHigher[0]):
            skinHigher = numpy.array([current[0]+10,current[1]+10,current[2]+20])
        
        if (current[0]<skinLower[0]):
            skinLower = numpy.array([current[0]-10,current[1]+10,current[2]-20])
    

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
        frame = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

        mask = cv.inRange(frame,numpy.array([100,60,60]),numpy.array([140, 255, 255]))
        
        edges = cv.Canny(mask,50,150)
        

        cv.imshow("Mask", mask)
        cv.imshow("edges", edges)
    
    # Se durante o periodo de configuração a tecla Ctrl for pressionada
    if color_configured==0 and cv.waitKey(10) == 115:
        # Fazer o histograma das cores da mão com base nos circulos definidos
        color_calibration(frame)

    if color_configured==0:
        draw_hand_circle(frame_circle)
        

    

        


    cv.imshow("Video", frame_circle)

    if cv.waitKey(10) == 27:
        break
