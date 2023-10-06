import cv2 as cv
import numpy as np

TOLERANCE = 15

def colorthresh(frame):

    #hsv:353,77,95 opencv:176.5,196.35,242.25
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([40, 255, 255])
    lower_red2 = np.array([160,100,100])
    upper_red2 = np.array([180,255,255])

    #213,78,88  106.5,198.9,224.4
    lower_blue = np.array([95,100,100])
    upper_blue = np.array([115,255,255])

    #310,44,63  155,112.2,160.65
    lower_purple = np.array([115,30,100])
    upper_purple = np.array([160,255,255])

    lower_black = np.array([0,0,0])
    upper_black = np.array([180,255,80])

    search_ratio = 0.3

    h, w, _ = np.shape(frame)
    
    frame = cv.GaussianBlur(frame, (3, 3), 0.1)

    frame = frame[int((1-search_ratio)*h):h, 0:w]
    frame = frame[0:h, int((1-search_ratio)*w):w]
    frame = frame[0:int(search_ratio*h), 0:w]
    frame = frame[0:h, 0:int(search_ratio*w)]

    #HSV変換
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    hsv_ave = [hsv.T[0].flatten().mean(),hsv.T[1].flatten().mean(),hsv.T[2].flatten().mean()]

    #print(hsv_ave)

    color = ""

    if((np.all(lower_red1 < hsv_ave) & np.all(hsv_ave < upper_red1)) | (np.all(lower_red2 < hsv_ave) & np.all(hsv_ave < upper_red2))):
        color = "red"
    elif(np.all(lower_blue < hsv_ave) & np.all(hsv_ave < upper_blue)):
        color = "blue"
    elif(np.all(lower_purple < hsv_ave) & np.all(hsv_ave < upper_purple)):
        color = "purple"
    elif(np.all(lower_black < hsv_ave) & np.all(hsv_ave < upper_black)):
        color = "black"
    else:
        color = "other"

    return color, frame, hsv

if __name__ == '__main__':
    cap = cv.VideoCapture(0)

    while(True):
        ret ,frame = cap.read()
    
        if ret == False:
            print("cap error")
        
        color, frame, hsv = colorthresh(frame)

        print (color)

        frame = cv.resize(frame,(640,480))

        cv.imshow('img',frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows() 
        