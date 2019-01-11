import cv2
import time
import numpy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30);

while(True):
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.bitwise_not(cv2.threshold(frame,127,255,cv2.THRESH_BINARY)[1])
    
    edges = cv2.Canny(frame,50,150,apertureSize = 3)
    minLineLength = 100
    maxLineGap = 10
    lines = cv2.HoughLinesP(edges,1,numpy.pi/180,100,minLineLength,maxLineGap)
    
    for x1,y1,x2,y2 in lines[0]:
        cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)#
    
    
    cv2.imshow('frame1', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

