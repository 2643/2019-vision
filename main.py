import cv2
import time
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
        dx = x1 - x2
        dy = y1 - y2
    else:
        dx = x2 - x1
        dy = y2 - y1


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

    WINDOW = True
    NETWORKING = False
    CONFIGURE = True

    NetworkTables.initialize(server="10.26.43.2")

    if NETWORKING:
        print("connecting to network");
        # Wait for networktables to connect 
        while not NetworkTables.isConnected():
            print("waiting to connect...")
            time.sleep(0.5)
            pass

        print("connected")
        visionTable = NetworkTables.getTable("vision")

    cap = cv2.VideoCapture(-1)

    if CONFIGURE:
        print("configuring camera")
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, 2)
        print("camera configuration complete")

    # get dimensions of video feed
    xsize = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    ysize = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) 


    while True:
        frame = cap.read()[1]
        hsv_thresh = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), 
                                 (113, 89, 0), 
                                 (123, 255, 80))

        closed = erode(hsv_thresh, (5,5), 2)
        closed = dilate(closed, (5,5), 2)

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
            cv2.drawContours(frame,[rect],0,(255,0,0),2)
            if slope > 0 and cv2.contourArea(rect) > 0:
                cv2.drawContours(frame,[rect],0,(0,0,255),2)
                rect1 = rect
                break

        for contour in contours:
            rect = getRect(contour)
            slope = getRectangleTiltSlope(rect)
            cv2.drawContours(frame,[rect],0,(255,0,0),2)
            if slope < -0 and cv2.contourArea(rect) > 0:
                cv2.drawContours(frame,[rect],0,(0,0,255),2)
                rect2 = rect
                break


        valid = False
        centroid_left_x = 0.0
        centroid_left_y = 0.0
        centroid_right_x = 0.0
        centroid_right_y = 0.0

        # If we have 2 rectangles and they're arranged in the right way
        if rect1 is not None and rect2 is not None and getCentroid(rect1)[0] < getCentroid(rect2)[0]:
            valid = True
            centroid1 = getCentroid(rect1)
            centroid2 = getCentroid(rect2)
            relative1 = getRelative(centroid1[0], centroid1[1], xsize, ysize)
            relative2 = getRelative(centroid2[0], centroid2[1], xsize, ysize)
            centroid_left_x = relative1[0]
            centroid_left_y = relative1[1]
            centroid_right_x = relative2[0]
            centroid_right_y = relative2[1]
        else:
            valid = False


        print("valid: ", valid, " l: ", centroid_left_x, " r: ", centroid_right_x)

        if NETWORKING:
            # Send the data to the computer
            visionTable.putNumber("centroid-left-x", centroid_left_x)
            visionTable.putNumber("centroid-left-y", centroid_left_y)
            visionTable.putNumber("centroid-right-x", centroid_right_x)
            visionTable.putNumber("centroid-right-y", centroid_right_y)
            visionTable.putBoolean("valid", valid)

        if WINDOW:
            cv2.imshow("raw_image", frame)
            cv2.imshow("processed_image", closed)
            if cv2.waitKey() == ord('q'):
                break
        


    # When everything done, release the capture
    cap.release()


main()
