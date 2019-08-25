#import numpy as np
import cv2
import time
import numpy as np
import logging
import logging.config
import yaml
import robot
import jsonobject
from shapely.geometry import Point, Polygon
import threading
import math

"""
Improvements on control:
* avoid stops
* estimate speed, and estimate when hitting the fence between frames
* track blob - with speed, select the blob that fits the extrapolated position
* include elliptic blob in selection
* watchdog on video frame analysis - when stops up/lags, then bail out / reset
* online control of parameters, e.g. sensitivity

* online dashboard to monitor video characteristics, etc
* monitor current (to cutter) and reset - regular reset!?
* missing heartbeat stops cutter too
 
"""
logger = logging.getLogger("main")

# BGR
Red = (0, 0, 255)
Blue = (255, 0, 0)
Black = (0, 0, 0)
White = (255, 255, 255)


def area_and_centroid(M):
    A = M["m00"]
    cX = int(M["m10"] / A)
    cY = int(M["m01"] / A)
    return A, cX, cY


def area(M):
    return M["m00"]


def draw_exterior(shape, image, color, width):
    pts = shape.exterior.coords[:]
    p0 = tuple(map(int, pts.pop(0)))
    while pts:
        p1 = tuple(map(int, pts.pop(0)))
        cv2.line(image, p0, p1, color, width)
        p0 = p1


# def draw_polyline(points, image, color, width):
def draw_polyline(image):
    pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(image, [pts], True, (0,255,255), 5, 0)


def blob_detector():
    # Set our filtering parameters
    # Initialize parameter settiing using cv2.SimpleBlobDetector
    params = cv2.SimpleBlobDetector_Params()

    # Set Area filtering parameters
    params.filterByArea = True
    params.minArea = 20

    # Set Circularity filtering parameters
    params.filterByCircularity = True
    params.minCircularity = 0.9

    # Set Convexity filtering parameters
    params.filterByConvexity = True
    params.minConvexity = 0.2

    # Set inertia filtering parameters
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    return detector


def detect_blobs(detector, frame):
    # Detect blobs
    keypoints = detector.detect(frame)
    # Draw blobs on our image as red circles
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(frame, keypoints, blank, (0, 0, 255),
                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    text = "Number of Circular Blobs: " + str(len(keypoints))
    cv2.putText(blobs, text, (20, 550), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2)
    cv2.imshow("Circular Blobs", blobs)


def main():
    # logging.basicConfig()
    with open("logging.yaml") as f:
        logging.config.dictConfig(yaml.full_load(f))

    logger.info("Starting video control")

    config = jsonobject.fromJson(yaml.full_load(open("config.yaml")))

    # define range of white color in HSV
    if config.video.blue:
        hsv_min = np.array([95, 85, 160])
        hsv_max = np.array([120, 255, 255])
    elif config.video.white:
        hsv_min = np.array([0, 0, 255 - config.video.sensitivity])
        hsv_max = np.array([255, config.video.sensitivity, 255])
    else:
        # print(config.video.color)
        hsv_min = np.array([x.min for x in config.video.color])
        hsv_max = np.array([x.max for x in config.video.color])

    # fence = box(580, 350, 1200, 600)
    # print(config.fence)
    fence = Polygon(list(((p.x, p.y) for p in config.fence)))
    aoi = fence.buffer(config.video.aoi_buffer, resolution=1, join_style=2)
    detector = blob_detector()

    logger.info("Connecting to robot...")
    robot.start(config.robot.url)

    logger.info("Starting camera loop...")

    recording = None
    course = None
    outside = True
    last_time = time.time()
    last_point = None
    avoid_complete = threading.Event()
    avoid_complete.set()

    while cv2.waitKey(1) & 0xFF != ord('q'):
        # SNAP!!!! Better with video????
        # Capture frame-by-frame
        cap = cv2.VideoCapture(config.video.url)
        ret, frame = cap.read()
        if not ret:
            logger.warning("No frame %r", ret)
            continue
        t = time.time()
        logger.debug("Frame interval %.3f", t - last_time)
        last_time = t

        if config.video.record and recording is None:
            import datetime
            timestamp = str(datetime.datetime.now())[:19].replace(":", "").replace(" ", "-")
            filename = 'recording/%s.avi' % timestamp
            logger.info("Recording video to %s", filename)
            height, width, layers = frame.shape
            recording = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'DIVX'), 15, (width, height))

        if recording:
            recording.write(frame)

        # blur before drawing anything in frame
        if config.video.blur:
            blur = cv2.GaussianBlur(frame, (config.video.blur, config.video.blur), 0)
        else:
            blur = frame

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
        mask = cv2.inRange(hsv, hsv_min, hsv_max)

        if config.video.erode.kernel:
            kernel = np.ones((config.video.erode.kernel, config.video.erode.kernel), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=config.video.erode.iterations or 1)

        if config.video.dilate.kernel:
            kernel = np.ones((config.video.dilate.kernel, config.video.dilate.kernel), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=config.video.dilate.iterations or 1)

        cv2.imshow('mask', mask)
        #res = cv2.bitwise_and(frame, frame, mask=mask)
        #cv2.imshow('res',res)

        def onMouse(event, x, y, flags, param):
            # print(event, flags, param)
            if event == cv2.EVENT_LBUTTONDOWN:
                # draw circle here (etc...)
                print('x = %d, y = %d' % (x, y))
                print("BRG", frame[y][x])
                print("HSV", hsv[y][x])
                for x, mm in zip(hsv[y][x], zip(hsv_min, hsv_max)):
                    xmin, xmax = mm
                    print('  ', x, xmin, xmax, xmin <= x <= xmax)
        cv2.setMouseCallback('frame', onMouse)

        # detect_blobs(detector, mask)

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

            if not aoi.contains(p):
                logger.debug("Invalid point %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
                cv2.circle(frame, (cX, cY), 15, Blue, 2)
                continue

            if not (config.video.area_min <= A <= config.video.area_max):
                logger.debug("Invalid area %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
                cv2.circle(frame, (cX, cY), 15, Red, 2)
                continue

            logger.debug("Valid area %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))

            # TODO: find the most elliptical contour, avoid the "rear end" of robot (for white)

            ellipse = cv2.fitEllipse(c)
            cv2.ellipse(frame, ellipse, Red, 2)


            ellipse_area = math.pi * ellipse[1][0] * ellipse[1][1] / 4
            logger.debug("Ellipse area %.2f diff %.2f", ellipse_area, ellipse_area - A)
            # TODO: find the center point of the ellipse
            # appears to be "rotated" rectangle - x,y = ellipse[0]
            # print(ellipse)

            break

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
                robot.avoid(config.robot.speed, config.robot.turnRate)
                # Only tries it once! So don't have to wait for it to complete.
                # Will only continue when if the avoid ends up inside
                continue

            # TODO: check if avoidance is fininshed...
            continue

            assert outside
            if fence.exterior.distance(p) < config.robot.max_outside_distance:
                robot.avoid(config.robot.speed, config.robot.turnRate)
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

        if robot.battery_level is not None:
            logger.info("Battery level %.3f", robot.battery_level)
            if robot.battery_level < config.robot.battery_cutoff:
                logger.warning("Battery low - stopping")
                robot.send(robot.Stop)
                continue

        logger.debug("Avoidance %r", avoid_complete.isSet())
        if avoid_complete.isSet():
            # this is a problem if just reversing out of fence, immediately goes forward.
            # but only from manual GUI control?
            # robot.send(robot.Speed(config.robot.speed))
            # else:
            # don't mess up the avoidance
            robot.send(robot.Heartbeat)
        else:
            logger.info("In avoidance")

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        

if __name__ == '__main__':
    main()