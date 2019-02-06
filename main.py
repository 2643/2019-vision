import cv2
import subprocess
import math
import numpy
from networktables import NetworkTables

def opencvVersion():
    return int(cv2.__version__.split(".")[0])

def erode(img, size, iterations):
    kernel = numpy.ones(size,numpy.uint8)
    return cv2.erode(img,kernel, iterations)

def dilate(img, size, iterations):
    kernel = numpy.ones(size,numpy.uint8)
    return cv2.dilate(img,kernel,iterations)

def open(img):
    opened = erode(img, (5,5), 2)
    opened = dilate(opened, (5,5), 2)
    return opened

def close(img):
    closed = dilate(img, (5,5), 2)
    closed = erode(closed, (5,5), 2)
    return closed

def convexHull(input_contours):
    output = []
    for contour in input_contours:
        output.append(cv2.convexHull(contour))
        return output

def getCentroid(contour):
    M = cv2.moments(contour)
    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
        return (cX, cY)

def slope(x1, y1, x2, y2):
    #swap if less
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1 

    return float(x2 -x1)/(y2 - y1)

def getRectangleTiltSlope(rect):
    if math.hypot(rect[0][0] - rect[1][0], rect[0][1] - rect[1][1]) < math.hypot(rect[1][0] - rect[2][0], rect[1][1] - rect[2][1]):
        return slope(rect[0][0], rect[0][1], rect[1][0], rect[1][1])
    else:
        return slope(rect[1][0], rect[1][1], rect[2][0], rect[2][1])


def getAngle(x, y, xsize, ysize):
    return ((float(x)/float(xsize)) -0.5, (float(y)/float(ysize)) -0.5)


def handleRectangle(contour, rectEntry): 
    
    boundingBox = numpy.int0(cv2.boxPoints(cv2.minAreaRect(contour)))

    rectEntry.putNumber("centroid", getCentroid(contour)[1])
    return

def main():

    NetworkTables.initialize(server="roborio-2643-frc.local")
    table = NetworkTables.getTable("vision")

    entry = table.

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 30)
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "exposure_auto=1"])
    subprocess.run(["v4l2-ctl", "--set-ctrl=exposure_absolute=6", "--device=/dev/video0"])

    while True:
        frame = cap.read()[1]
        hsv_thresh = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), (120, 250, 3), (130, 255, 80))
        closed = close(hsv_thresh)
        if opencvVersion() == 3:
            _, contours, _ = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE);
        else:
            contours, _ = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


        hulls = convexHull(contours)
        for contour in hulls: 
            if cv2.contourArea(contour) > 100:
                handleRectangle(contour, entry)

        #cv2.imshow('frame3', closed)
        #cv2.imshow('frame2', frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break
            # When everything done, release the capture
            cap.release()
            cv2.destroyAllWindows()


main()

