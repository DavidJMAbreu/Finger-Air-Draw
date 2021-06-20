import cv2 as cv
import numpy
from collections import deque


# Carregar o video da camêra principal
webcam = cv.VideoCapture(0)

# Variáveis de projeto
global color_configured
color_configured = 0
global skinLower, skinHigher
skinLower = numpy.array([255, 255, 255])
skinHigher = numpy.array([0, 255, 255])


# Variáveis para armazenar os pixeis desenhados
pontos_azul = [deque(maxlen=512)]
pontos_verde = [deque(maxlen=512)]
pontos_vermelho = [deque(maxlen=512)]
pontos_preto = [deque(maxlen=512)]
pontos_amarelo = [deque(maxlen=512)]
# Indices para as cores
blindex = 0
bindex = 0
gindex = 0
rindex = 0
yindex = 0
# Vetor com as cores a utilizar
# preto - azul - verde - vermelho - amarelo
cores = [(0,0,0),(255,0,0),(0,255,0),(0,0,255),(0,255,255)]
cor_atual = 0

#Tela branca
tela = numpy.zeros((480,320,3))+255
tela = cv.rectangle(tela, (10,1), (52,50), (0,0,0), 2)
tela = cv.rectangle(tela, (62,1), (104,50), cores[0], -1)
tela = cv.rectangle(tela, (114,1), (156,50), cores[1], -1)
tela = cv.rectangle(tela, (166,1), (208,50), cores[2], -1)
tela = cv.rectangle(tela, (218,1), (260,50), cores[3], -1)
tela = cv.rectangle(tela, (270,1), (312,50), cores[4], -1)
cv.putText(tela, "CL", (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv.LINE_AA)
cv.putText(tela, "0", (25, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv.LINE_AA)
cv.putText(tela, "1", (77, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(tela, "2", (129, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(tela, "3", (181, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
cv.putText(tela, "4", (233, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 2, cv.LINE_AA)
cv.putText(tela, "5", (285, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 2, cv.LINE_AA)





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

    color_configured = 1


def doNothing(x):
    pass

def getPoint(contour):
    #Obter as linhas para descobrir a mais alta
    lines = []
    points = []
    index = 0
    for c in contour:
        lines.append(c[0][1])
    
    highest = min(lines)
    while(index<len(lines)):
        try:
            new_find = lines.index(highest,index)
            points.append(new_find)
            index = new_find+1
        except ValueError as e:
            break        

    #Obter as colunas e fazer a média entre todos
    col = 0
    for point in points:
        col += contour[point][0][0]
    col = round(col/len(points))

    return [highest,col]

def savePoint(point):
    if cor_atual == 0:
        pontos_preto[blindex].appendleft(point)
    elif cor_atual == 1:
        pontos_azul[bindex].appendleft(point)
    elif cor_atual == 2:
        pontos_verde[gindex].appendleft(point)
    elif cor_atual == 3:
        pontos_vermelho[rindex].appendleft(point)
    elif cor_atual == 4:
        pontos_amarelo[yindex].appendleft(point)

def draw(tela):
    points = [pontos_preto,pontos_azul,pontos_verde,pontos_vermelho,pontos_amarelo]
    for color in range(len(points)):
        for deque in range(len(points[color])):
            for point in range(len(points[color][deque])):
                if points[color][deque][point - 1] is None or points[color][deque][point] is None:
                    continue
                cv.line(tela, points[color][deque][point - 1], points[color][deque][point], cores[cor_atual], 2)
                print(points[color][deque][point])
            


    


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

        cv.imshow("media",medianFilter)
        

        contours, hierarchy = cv.findContours(medianFilter, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
         
        
        if contours != []:
            # Detetar qual a maior àrea
            largest = 0
            max_area = 0
            for contour in contours:
                area = cv.contourArea(contour)
                if (area>max_area):
                    max_area = area
                    largest = contour

            column,line = getPoint(largest)

            cv.circle(frame[0:480,round(col/2):col,0:3],(line,column+15),2,cores[cor_atual],-1)
            cv.circle(tela,(line,column+15),2,cores[cor_atual],-1)

            savePoint([line,column])
                
                    

    # Se durante o periodo de configuração a tecla Ctrl for pressionada
    if color_configured == 0 and cv.waitKey(1) == 115:
        # Fazer o histograma das cores da mão com base nos circulos definidos
        color_calibration(frame)

    if color_configured == 1:
        if cv.waitKey(1) == 114:
            color_configured = 0
        else:
            key = cv.waitKey(1)
            if key != -1:
                if key == 48:
                    pontos_preto = [deque(maxlen=512)]
                    pontos_azul = [deque(maxlen=512)]
                    pontos_verde = [deque(maxlen=512)]
                    pontos_vermelho = [deque(maxlen=512)]
                    pontos_amarela = [deque(maxlen=512)]

                    blindex = 0
                    bindex = 0
                    gindex = 0
                    rindex = 0
                    yindex = 0
                    tela = numpy.zeros((480,320,3))+255
                    tela = cv.rectangle(tela, (10,1), (52,50), (0,0,0), 2)
                    tela = cv.rectangle(tela, (62,1), (104,50), cores[0], -1)
                    tela = cv.rectangle(tela, (114,1), (156,50), cores[1], -1)
                    tela = cv.rectangle(tela, (166,1), (208,50), cores[2], -1)
                    tela = cv.rectangle(tela, (218,1), (260,50), cores[3], -1)
                    tela = cv.rectangle(tela, (270,1), (312,50), cores[4], -1)
                    cv.putText(tela, "CL", (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv.LINE_AA)
                    cv.putText(tela, "0", (25, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv.LINE_AA)
                    cv.putText(tela, "1", (77, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
                    cv.putText(tela, "2", (129, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
                    cv.putText(tela, "3", (181, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
                    cv.putText(tela, "4", (233, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 2, cv.LINE_AA)
                    cv.putText(tela, "5", (285, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 2, cv.LINE_AA)
                else:
                    if key == 49:
                        cor_atual = 0
                    else:
                        if key == 50:
                            cor_atual = 1
                        else:
                            if key == 51:
                                cor_atual = 2
                            else:
                                if key == 52:
                                    cor_atual = 3
                                else:
                                    if key == 53:
                                        cor_atual = 4
            

    if color_configured == 0:
        draw_hand_circle(frame_circle)

    cv.imshow("Video", frame_circle)
    draw(tela)
    cv.imshow("Tela",tela)

    if cv.waitKey(1) == 27:
        break
