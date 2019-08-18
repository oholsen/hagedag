#import numpy as np
import cv2
import time
import numpy as np
import logging
import logging.config
import yaml
import robot
import jsonobject
from shapely.geometry import Point, box


logger = logging.getLogger("main")


def area_and_centroid(M):
    A = M["m00"]
    cX = int(M["m10"] / A)
    cY = int(M["m01"] / A)
    return A, cX, cY

def area(M):
    return M["m00"]


kernel = np.ones((9,9),np.uint8)


def draw_exterior(shape, image, color, width):
    pts = shape.exterior.coords[:]
    p0 = tuple(map(int, pts.pop(0)))
    while pts:
        p1 = tuple(map(int, pts.pop(0)))
        cv2.line(image, p0, p1, color, width)
        p0 = p1

#def draw_polyline(points, image, color, width):
def draw_polyline(image):
    pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(image, [pts], True, (0,255,255), 5, 0)


def main():
    #logging.basicConfig()
    with open("logging.yaml") as f:
        logging.config.dictConfig(yaml.full_load(f))

    config = jsonobject.fromJson(yaml.full_load(open("config.yaml")))

    # TODO: config below....
    # define range of white color in HSV
    lower_white = np.array([0,0,255-config.video.sensitivity])
    upper_white = np.array([255,config.video.sensitivity,255])

    lower_blue = np.array([95, 85, 20])
    upper_blue = np.array([120, 255, 255])

    # TODO: config white or blue - or add to config...
    #lower_hsv, upper_hsv = lower_blue, upper_blue
    lower_hsv, upper_hsv = lower_white, upper_white

    fence = box(580, 350, 1200, 600)
    aoi = fence.buffer(config.video.aoi_buffer, resolution=1, join_style=2)


    logger.info("Connecting to robot...")
    robot.start("ws://gardenbot.local:8000/control")

    logger.info("Starting camera loop...")
    course = None
    outside = True
    last_time = time.time()
    last_point = None

    while cv2.waitKey(1) & 0xFF != ord('q'):
        # SNAP!!!! Better with video????
        cap = cv2.VideoCapture(config.video.url)
        t = time.time()

        # Capture frame-by-frame
        ret, frame = cap.read()    
        logger.debug("Frame interval %.3f", t - last_time)
        if not ret:
            logger.warning("No frame %r", ret)
            continue
        last_time = t

        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        draw_exterior(fence, frame, (0, 0, 255), 2)
        draw_exterior(aoi, frame, (0, 0, 0), 2)
        #draw_polyline(frame)
        cv2.imshow('frame', frame)

        if course is None:
            course = frame.copy()
            cv2.imshow('course', course)

        if 0:
            # downsample
            width = int(blur.shape[1] / 2)
            height = int(blur.shape[0] / 2)
            dim = (width, height)
            blur = cv2.resize(blur, dim, interpolation = cv2.INTER_AREA)
 
        # cv2.imshow('blur', blur)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)    
        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        cv2.imshow('mask',mask)
        #res = cv2.bitwise_and(frame, frame, mask=mask)
        #cv2.imshow('res',res)

        def onMouse(event, x, y, flags, param):
            # print(event, flags, param)
            if event == cv2.EVENT_LBUTTONDOWN:
               # draw circle here (etc...)
               print('x = %d, y = %d' % (x, y))
               print(frame[y][x], hsv[y][x])
        #cv2.setMouseCallback('frame', onMouse)


        #contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            logger.info("No contours")
            continue
        # the contours are drawn here
        #cv2.drawContours(mask, contours, -1, 255, 3)
        #cv2.imshow("mask", mask)
        # find the contour with the biggest area
        contours.sort(key = cv2.contourArea)
        contours.reverse()
        for c in contours:
            #print(cv2.contourArea(c))
            M = cv2.moments(c)
            try:
                A, cX, cY = area_and_centroid(M)
            except:
                # logger.info("No point")
                continue
            if 0:
                # correct for downsampling
                cX *= 2
                cY *= 2
                A *= 4

            p = Point(cX, cY)

            if (config.video.area_min <= A <= config.video.area_max) and aoi.contains(p):
                logger.debug("Valid area %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
                break

            logger.debug("Invalid area %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
            cv2.circle(frame, (cX, cY), 15, (0, 0, 255), 2)

        else:
            logger.info("No point")
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

        t = time.time()

        # Buffer zone outside the fence: will run avoidance maneouvre.
        # Get into an outside state, if staying outside too long, then abort
        if not fence.contains(p):

            if not outside:
                logger.info("OUTSIDE")
                outside = True
                robot.avoid(config.robot.speed, config.robot.turn)
                # Only tries it once! So don't have to wait for it to complete.
                # Will only continue when if the avoid ends up inside
                continue

            # TODO: check if avoidance is fininshed...
            continue

            assert outside
            if fence.exterior.distance(p) < config.robot.max_outside_distance:
                robot.avoid(config.robot.speed, config.robot.turn)
                # Only tries it once! So don't have to wait for it to complete.
                # Will only continue when if the avoid ends up inside
                continue

            continue

        # inside fence
        if outside:
            logger.info("INSIDE")
            time_into_inside = t
            #robot.send(robot.Speed(speed))
            outside = False
            #continue

        assert not outside
        if t > time_into_inside + config.robot.inside_timeout:
            logger.warning("INSIDE timeout")
            robot.send(robot.Stop)
            continue

        robot.send(robot.Heartbeat)


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        

if __name__ == '__main__':
    main()