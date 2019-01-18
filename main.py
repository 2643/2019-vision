import cv2
import math
import numpy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30);
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0);
# cap.set(cv2.CAP_PROP_EXPOSURE, 0.5);
# cap.set(cv2.CAP_PROP_GAIN, 0.5);

while True:
    frame = cap.read()[1]
    blur = cv2.medianBlur(frame, 21)
    hsv_thresh = cv2.inRange(cv2.cvtColor(blur, cv2.COLOR_BGR2HSV), (0, 0, 180), (180, 60, 255))
    cv2.imshow('frame3', hsv_thresh)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

