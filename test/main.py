
from py_image_search.shape_detector import ShapeDetector
import threading
import time
import cv2
import numpy as np

list_x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
list_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

run = 0

def nothing(x):
    pass

#track mau
def tao_track():
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("H_min", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("S_min", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("V_min", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("H_max", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("S_max", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("V_max", "Trackbars", 255, 255, nothing)

data = ""
H_max = 0
H_min =0
S_max =0
S_min=0
V_max=0
V_min=0
sd = ShapeDetector()

#xu ly mau
def nhan_dang_mau(img_camera):
    global H_min, S_min, V_min
    global H_max, S_max, V_max, data

    ret_val, image = img_camera
    # lam mo loc nhieu anh
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # range color
    lower_red = np.array([H_min, S_min, V_min])
    upper_red = np.array([H_max, S_max, V_max])
    thresh = cv2.inRange(hsv, lower_red, upper_red) #lay cac gia tri trong anh nay
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    for contour in contours:
        M = cv2.moments(contour)
        cxa = int(M["m10"]/M["m00"])
        cya = int(M["m01"]/M["m00"])
        shape = sd.detect(contour)
    # for contour in contours: #nhan dien hinh hoc
        # x, y, w, h = cv2.boundingRect(contour)
    #     if w*h > 1000:
    #         M = cv2.moments(contour)
    #         cxa = int(M["m10"]/M["m00"])
    #         cya = int(M["m01"]/M["m00"])
        list_x[cnt] = cxa
        list_y[cnt] = cya
        # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0))
        # cv2.circle(image, (cxa, cya), 3, (100, 150, 255), -1)
        # cv2.putText(image, str(list_x[cnt])+" "+str(list_y[cnt]), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
        cv2.putText(image, shape, (cxa, cya), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)

    cv2.imshow("image", image)
    #cv2.imshow("thresh", thresh)
    cv2.imshow("mask", mask)
    #cv2.imshow("hsv", hsv)

def show_camera():
    global H_min, S_min, V_min
    global H_max, S_max, V_max

    cap = cv2.VideoCapture(0)

    #-------Tao track-------#
    tao_track()

    if cap.isOpened():
        #window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        while True:

            #----------Lay HSV-----------#
            # lay_hsv()
            H_min = cv2.getTrackbarPos("H_min", "Trackbars")
            S_min = cv2.getTrackbarPos("S_min", "Trackbars")
            V_min = cv2.getTrackbarPos("V_min", "Trackbars")
            H_max = cv2.getTrackbarPos("H_max", "Trackbars")
            S_max = cv2.getTrackbarPos("S_max", "Trackbars")
            V_max = cv2.getTrackbarPos("V_max", "Trackbars")

            nhan_dang_mau(cap.read())
            # a = ser.readline()
            # print(a)
         
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                run = 1
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

#============Send_data===============#

def send_data():
    while True:
        time.sleep(0.2)
        print("----------------")
        if run == 1:
            print("GOOD BYE!")
            break
        # ser.write(data.encode())
    
if __name__ == "__main__":
    try:
        t1 = threading.Thread(target = show_camera, args=())
        t2 = threading.Thread(target = send_data, args=())
        t1.start()
        t2.start()
        t1.join()
        t2.join()

    except:
        print("----error----")
    #show_camera()



