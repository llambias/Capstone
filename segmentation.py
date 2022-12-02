import cv2
import numpy as np
import time
from pid import Pid

#https://www.geeksforgeeks.org/realtime-distance-estimation-using-opencv-python/#:~:text=Steps%20for%20Distance%20Estimation%3A,Mine%20Reference%20Image

def FocalLength(measured_distance, real_width, width_in_rf_image):
    focal_length = (width_in_rf_image* measured_distance)/ real_width
    return focal_length

def Distance_finder(Focal_Length, real_width, width_in_frame):
    distance = (real_width * Focal_Length)/width_in_frame
    return distance

cam = cv2.VideoCapture(0)

cv2.namedWindow("test")
cv2.namedWindow("original")

img_counter = 0

focal_length = 1554
i = 0
while True:
    ret, frame = cam.read()

    if not ret:
        print("failed to grab frame")
        break

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gaussian_blurr = cv2.GaussianBlur(gray_frame, (25, 25), 0)

    # (source, method, dp(mientras más grande, junta circulos cercanos), distancia mínima entre círculos, 
    # param1: sensibilidad (muy alto y no encuentra, muy bajo encuentra muchos), param2: 30 (número de puntos necesarios para decir que hay un círculo))
    # min_radius: 20, 
    circles = cv2.HoughCircles(gaussian_blurr, cv2.HOUGH_GRADIENT, 1.2, 100,
                                param1=80, param2=50, minRadius=5, maxRadius=200)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        i = 0
        for circle in circles[0]:

           
            img_circle = frame.copy()
            mask = np.zeros_like(gray_frame)
            (x,y,r) = circle
            x = int(x)
            y = int(y)
            r = int(r)
            cv2.circle(img_circle, (x, y), r, (0, 0, 255), 2)
            cv2.circle(mask, (x, y), r, 255, -1)

            # image, center_coordinates, radius, color, thickness
            #cv2.circle(gaussian_blurr, (circle[0], circle[1]), circle[2], (255,0,255), 3)
            pixel_centro = frame[y][x]
            ave_color = cv2.mean(frame, mask=mask)[:3]
            #print(f'average circle color: {ave_color}')
            average = (ave_color[0] + ave_color[1] + ave_color[2]) / 3
            color_centro = (pixel_centro[0] + pixel_centro[1] + pixel_centro[2]) / 3
            if average < 180:
                break

            cv2.circle(frame, (circle[0], circle[1]), circle[2], (255,0,255), 3)
            cv2.circle(frame, (circle[0], circle[1]), 2, (0,0,0), 3)


            if focal_length is None:
                focal_length = FocalLength(42, 4.27, circle[2]*2)
                print(f'Focal length: {focal_length}')
            
            distance = Distance_finder(focal_length, 4.27, circle[2]*2)

            # Distancia del centro a pelota en pixeles
            dist_centro_pixeles = circle[0] - cam.get(cv2.CAP_PROP_FRAME_WIDTH)/2

            # diámetro en cm / diámetro en pixeles
            ponderador = 4.27 / (2*circle[2])

            # Se multiplica por ponderador
            dist_centro_cm = dist_centro_pixeles * ponderador

            # Se obtiene el ángulo
            angulo = np.degrees(np.arcsin(dist_centro_cm / distance))

            cv2.putText(frame, f"Distance {i}: {round(distance,2)} CM  -  Angulo {i}: {round(angulo,2)}deg", (30, 35*(i)), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 0), 2)

            pid_ang = Pid(1.9, 0.1, 0)
            pid_dis = Pid(1.9, 0.1, 0)

            pid_ang.error(angulo)
            pid_dis.error(dist_centro_cm)
            print(pid_ang.output())
            print(pid_dis.output())
            print('----------------')


    #cv2.imshow("test", gaussian_blurr)
    cv2.imshow("original", frame)
    i+=1

cam.release()

cv2.destroyAllWindows()


