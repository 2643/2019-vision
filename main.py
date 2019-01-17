import cv2
import math
import numpy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30);

while True:
    frame = cap.read()[1]

    cv2.imshow('frame3', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

