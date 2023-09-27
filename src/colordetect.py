import cv2 as cv
import numpy as np

TOLERANCE = 15

def colorthresh(frame):

    h, w, _ = np.shape(frame)
    
    frame = cv.GaussianBlur(frame, (3, 3), 0.1)
    
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    mask[0:int(0.7*h), 0:w] = 0
    
    contours, hierarchy = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        contour = contours[-1]
        cv.drawContours(frame, [contour], 0, (0, 255, 0), 3)
    
    mask[0:h, 0:int(0.3*w)] = 0
    mask[0:h, int(0.7*w):w] = 0
    
    res = ''
    m = cv.moments(mask)
    if m['m00'] == 0:
        res = 'search'
    else:
        c = (m['m10'] / m['m00'], m['m01'] / m['m00'])
        cv.circle(frame, tuple(int(x) for x in c), 5, (155, 200, 0), -1) 
    
        if c[0] < w/2 - TOLERANCE:
            res = 'turn left'
        elif c[0] > w/2 + TOLERANCE:
            res = 'turn right'
        else:
            res = 'go straight'

    return res, frame, hsv, mask

if __name__ == '__main__':
    frame = cv.imread("../images/img.png")
    #frame = cv.imread(cv.samples.findFile("starry_night.jpg"))

    res, frame, hsv, mask = colorthresh(frame)

    print(res)
    
    cv.imshow("frame", frame)
    cv.imshow("hsv", hsv)
    cv.imshow("mask", mask)
    
    k = cv.waitKey(0) & 0xFF
    
    
