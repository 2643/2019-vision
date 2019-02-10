import cv2
import time
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

def getRect(input_contour):
    return numpy.int0(cv2.boxPoints(cv2.minAreaRect(input_contour)))

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

    dx = float(x2 - x1)
    dy = float(y2 - y1)

    if dx == 0:
        return 0
    else: 
        return dy/dx

def getRectangleTiltSlope(rect):
    if math.hypot(rect[0][0] - rect[1][0], rect[0][1] - rect[1][1]) < math.hypot(rect[1][0] - rect[2][0], rect[1][1] - rect[2][1]):
        return slope(rect[0][0], rect[0][1], rect[1][0], rect[1][1])
    else:
        return slope(rect[1][0], rect[1][1], rect[2][0], rect[2][1])


def getRelative(x, y, xsize, ysize):
    return ((float(x)/float(xsize)) -0.5, (float(y)/float(ysize)) -0.5)

def main():

    NetworkTables.initialize(server="10.26.43.2")

    # Wait for networktables to connect 
    while not NetworkTables.isConnected():
        print("waiting to connect...")
        time.sleep(0.5)
        pass

    visionTable = NetworkTables.getGlobalTable()

    # config camera
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "exposure_auto=1"])
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "exposure_absolute=15"])
    cap = cv2.VideoCapture(-1)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # get dimensions of video feed
    xsize = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float
    ysize = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) # float

    while True:
        frame = cap.read()[1]
        hsv_thresh = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), (112, 92, 0), (149, 255, 90))
        closed = close(hsv_thresh)
        if opencvVersion() == 3:
            _, contours, _ = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE);
        else:
            contours, _ = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        rect1 = None
        rect2 = None
        contours.sort(key = lambda x: cv2.contourArea(x), reverse=True)


        # decide on right rect and left rect (it's the largest one with a slope in the right direction)
        for contour in contours: 
            rect = getRect(contour)
            slope = getRectangleTiltSlope(rect)
            if slope > 0:
                cv2.drawContours(frame,[rect],0,(0,0,255),2)
                rect1 = rect
                break
        for contour in contours:
            rect = getRect(contour)
            slope = getRectangleTiltSlope(rect)
            if slope < 0:
                cv2.drawContours(frame,[rect],0,(0,0,255),2)
                rect2 = rect
                break

        # If we have 2 rectangles and they're arranged in the right way
        if rect1 is not None and rect2 is not None and getCentroid(rect1)[0] < getCentroid(rect2)[0]:
            centroid1 = getCentroid(rect1)
            centroid2 = getCentroid(rect2)
            relative1 = getRelative(centroid1[0], centroid1[1], xsize, ysize)
            relative2 = getRelative(centroid2[0], centroid2[1], xsize, ysize)
            visionTable.putNumber("centroid-left-x", relative1[0])
            visionTable.putNumber("centroid-left-y", relative1[1])
            visionTable.putNumber("centroid-right-x", relative2[0])
            visionTable.putNumber("centroid-right-y", relative2[1])
            visionTable.putBoolean("valid", True)
        else:
            visionTable.putBoolean("valid", False)


        print("finished cycle")
        cv2.imshow("ok", frame)
        cv2.imshow("o2k", closed)
        if cv2.waitKey() ==  ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


main()
