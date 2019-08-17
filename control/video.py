#import numpy as np
import cv2
import time
import numpy as np
import logging
import logging.config
import yaml
#import util

sensitivity = 50
fence_distance = 20
speed = 6
turn = 10


def centroid(M):
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX, cY

def area(M):
    return M["m00"]

#logging.basicConfig()
with open("logging.yaml") as f:
    logging.config.dictConfig(yaml.full_load(f))

logger = logging.getLogger()



#from matplotlib import pyplot as plt

#import cv2.aruco as aruco
#aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
#parameters = aruco.DetectorParameters_create()

kernel = np.ones((9,9),np.uint8)
# define range of white color in HSV
# change it according to your need !
lower_white = np.array([0,0,255-sensitivity])
upper_white = np.array([255,sensitivity,255])


from shapely.geometry import Point, box
fence = box(580, 350, 1200, 600)
print(fence.exterior.coords)

logger.info("Connecting to robot...")
import robot3 as robot
robot.start("ws://gardenbot.local:8000/control")



logger.info("Starting camera loop...")
course = None
outside = True
last_time = time.time()
last_point = None

while cv2.waitKey(1) & 0xFF != ord('q'):
    # SNAP!!!! Better with video????
    cap = cv2.VideoCapture("http://192.168.1.118:88/cgi-bin/CGIProxy.fcgi?cmd=snapPicture2&usr=admin&pwd=Foscam!!&.mjpg")
    t = time.time()

    # Capture frame-by-frame
    ret, frame = cap.read()    
    logger.debug("Frame interval %.3f", t - last_time)
    if not ret:
        logger.warning("No frame %r", ret)
        continue
    last_time = t


    pts = fence.exterior.coords[:]
    p0 = tuple(map(int, pts.pop(0)))
    while pts:
        p1 = tuple(map(int, pts.pop(0)))
        cv2.line(frame, p0, p1, (0, 255, 0), 2)
        p0 = p1
    #cv2.rectangle(frame, (580, 350), (1200, 600), (0, 255, 0), 2)

    cv2.imshow('frame', frame)
    if course is None:
        # logger.info("Set course background %r", frame)
        course = frame.copy() #cv2.copyMakeBorder(frame,0,0,0,0,cv2.BORDER_REPLICATE)
        cv2.imshow('course', course)

    if 0:
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('gray', gray)

        blur = cv2.GaussianBlur(gray, (21, 21), 0)
        cv2.imshow('blur', blur)


        #th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
        #cv2.imshow('Adaptive threshold', th)
        retval, th2 = cv2.threshold(blur, 230, 255, cv2.THRESH_BINARY)
        cv2.imshow('Binary threshold', th2)


        opening = cv2.morphologyEx(th2, cv2.MORPH_OPEN, kernel)
        cv2.imshow('opening', opening)

        if 0:
            hist = cv2.calcHist([gray],[0],None,[256],[0,256])
            print(hist)
            plt.plot(hist)
            plt.show()

    if 0:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for c, i in zip(corners, ids):
                print(i, c, flush=True)

        print(corners, ids, flush=True)
        gray = aruco.drawDetectedMarkers(gray, corners)
        cv2.imshow('frame', gray)

    if 1:
        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        # cv2.imshow('blur', blur)

        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)    
        if 0:
            hist = cv2.calcHist( [hsv], [0, 1], None, [180, 256], [0, 180, 0, 256] )
            plt.imshow(hist, interpolation = 'nearest')
            plt.show()

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
        cv2.imshow('mask',mask)
        #res = cv2.bitwise_and(frame, frame, mask=mask)
        #cv2.imshow('res',res)

        if 1:
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            #print(contours)
            #print(hierarchy)
            if len(contours) == 0:
                logger.info("No contours")
                continue
            # the contours are drawn here
            cv2.drawContours(mask, contours, -1, 255, 3)
            cv2.imshow("mask", mask)
            # find the contour with the biggest area
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            try:
                cX, cY = centroid(M)
            except:
                logger.info("No point")
                continue
            #x,y,w,h = cv2.boundingRect(c)
            #print("bound", x,y,w,h)
            #output = frame.copy()
            #cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
            #cv2.imshow("Contours", output)

        else:
            # calculate moments of binary image
            M = cv2.moments(mask)
            #logger.info("Moments %r", M)
            logger.info("Moments %.2f  %.2f", M["nu02"], M["nu20"])
            if not M["m00"]:
                logger.info("No point")
                # Stop robot after some timeout....
                continue
            # calculate x,y coordinate of center
            cX, cY = centroid(M)

            # put text and highlight the center
        cv2.circle(frame, (cX, cY), 25, (0, 0, 0), 2)
        cv2.imshow("frame", frame)


        p = Point(cX, cY)
        logger.info("point %d %d %r %.2f", cX, cY, fence.contains(p), fence.exterior.distance(p))

        new_point = (cX, cY)
        #cv2.circle(course, (cX, cY), 2, (0, 0, 0), -1)
        if last_point:
            cv2.line(course, last_point, new_point, (10,10,10), 2)
        last_point = new_point
        cv2.imshow("course", course)

        # Buffer zone outside the fence: will run avoidance maneouvre.
        # Get into an outside state, if staying outside too long, then abort
        if not fence.contains(p):

            if not outside:
                logger.info("OUTSIDE")
                outside = True
                robot.avoid(speed, turn)
                # Only tries it once! So don't have to wait for it to complete.
                # Will only continue when if the avoid ends up inside
                continue

            # TODO: check if avoidance is fininshed...
            continue

            assert outside
            if fence.exterior.distance(p) < fence_distance:
                robot.avoid(speed, turn)
                # Only tries it once! So don't have to wait for it to complete.
                # Will only continue when if the avoid ends up inside
                continue

            continue

        # inside fence
        if outside:
            logger.info("INSIDE")
            #robot.send(robot.Speed(speed))
            outside = False
            #continue

        robot.send("heartbeat")



# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
    

