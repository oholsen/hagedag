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
A_min = 100
A_max = 1000


def centroid(M):
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX, cY

def area(M):
    return M["m00"]

logger = logging.getLogger()


kernel = np.ones((9,9),np.uint8)
# define range of white color in HSV
lower_white = np.array([0,0,255-sensitivity])
upper_white = np.array([255,sensitivity,255])


from shapely.geometry import Point, box
fence = box(580, 350, 1200, 600)


def main():
    import sys
    capurl = sys.argv[1]
    print(capurl)

    #logging.basicConfig()
    with open("logging.yaml") as f:
        logging.config.dictConfig(yaml.full_load(f))
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
        cap = cv2.VideoCapture(capurl)
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

        cv2.imshow('frame', frame)

        if course is None:
            course = frame.copy()
            cv2.imshow('course', course)

        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        # cv2.imshow('blur', blur)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)    
        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
        #cv2.imshow('mask',mask)
        #res = cv2.bitwise_and(frame, frame, mask=mask)
        #cv2.imshow('res',res)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            logger.info("No contours")
            continue
        # the contours are drawn here
        #cv2.drawContours(mask, contours, -1, 255, 3)
        #cv2.imshow("mask", mask)
        # find the contour with the biggest area
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)

        try:
            cX, cY = centroid(M)
        except:
            logger.info("No point")
            continue

        p = Point(cX, cY)
        A = area(M)

        if not (A_min <= A <= A_max):
            logger.info("Invalid area %d %d %r %.2f %.2f", cX, cY, fence.contains(p), fence.exterior.distance(p), A)
            cv2.circle(frame, (cX, cY), 25, (255, 255, 0), 2)
            cv2.imshow("frame", frame)
            continue

        cv2.circle(frame, (cX, cY), 25, (0, 0, 0), 2)
        cv2.imshow("frame", frame)

        logger.info("Point %d %d %r %.2f %.2f", cX, cY, fence.contains(p), fence.exterior.distance(p), A)

        new_point = (cX, cY)
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
        

if __name__ == '__main__':
    main()