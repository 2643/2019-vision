import cv2
import math
import numpy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30);

while True:
    frame = cap.read()[1]
    blur = cv2.medianBlur(frame, 21)
    hsv_thresh = cv2.inRange(cv2.cvtColor(blur, cv2.COLOR_BGR2HSV), (0, 0, 180), (180, 60, 255))
    edge = cv2.Canny(hsv_thresh, 0,0,3)
    edge = cv2.dilate(edge, numpy.ones((5,5),numpy.uint8))

    minLineLength = 100
    maxLineGap = 10
    lines = cv2.HoughLinesP(edge,1,numpy.pi/180,100,minLineLength,maxLineGap)
    
    if lines is not None and len(lines) is not 0:
        
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),3)
        
        midpointaverage_x_running = 0
        midpointaverage_y_running = 0
        angleaverage_running = 0
        for line in lines:
            for x1,y1,x2,y2 in line:
                midpointaverage_x_running += (x1 + x2)/2
                midpointaverage_y_running += (y1 + y2)/2
                angle = math.atan2((y2-y1),(x2-x1))
                print(math.degrees(angle))
                angleaverage_running += angle
        angle = angleaverage_running/len(lines)
        midpointaverage_x = math.trunc(midpointaverage_x_running/len(lines))
        midpointaverage_y = math.trunc(midpointaverage_y_running/len(lines))
        midpointaverage_x_2 = math.trunc(midpointaverage_x+200*math.cos(angle))
        midpointaverage_y_2 = math.trunc(midpointaverage_y+200*math.sin(angle))

        cv2.line(frame, (midpointaverage_x,midpointaverage_y), (midpointaverage_x_2,midpointaverage_y_2),30)
        
    
    
    cv2.imshow('frame1', hsv_thresh)
    cv2.imshow('frame2', frame)
    cv2.imshow('frame3', edge)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

