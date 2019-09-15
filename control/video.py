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
from colors import *
import diff

"""
Improvements on control:
* average multiple frames (robot is slow) to filter out video noise, eg in wind
* track motion, filter out positions far away from expected/current position + speed circle
* estimate speed, and estimate when hitting the fence between frames - factor in estimated latency in frame
* watchdog on video frame analysis - when stops up/lags, then bail out / reset
* online control of parameters without having to restart - e.g. re-read config
* Can we monitor latency? Flashing LED on robot? Track time in camera OSD. Frame metadata in stream?
  Known camera frame rate -> latency

* online dashboard to monitor video characteristics, etc
* monitor current (to cutter) and reset - regular reset!?
* missing heartbeat stops cutter too
 
"""

logger = logging.getLogger("main")


def empty_image(cap):
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # fps = int(cap.get(cv2.CAP_PROP_FPS))
    # n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # print(w, h, fps, n_frames)
    return np.zeros(shape=[h, w, 3], dtype=np.uint8)


def area_and_centroid(M):
    A = M["m00"]
    cX = int(M["m10"] / A)
    cY = int(M["m01"] / A)
    return A, cX, cY


def draw_exterior(shape, image, color, width):
    pts = shape.exterior.coords[:]
    p0 = tuple(map(int, pts.pop(0)))
    while pts:
        p1 = tuple(map(int, pts.pop(0)))
        cv2.line(image, p0, p1, color, width)
        p0 = p1


def onMouse(event, x, y, flags, param):
    # logger.info("onMouse %r %d %d", event, x, y)
    # print(event, flags, param)
    if event == cv2.EVENT_LBUTTONDOWN:
        # draw circle here (etc...)
        print('x = %d, y = %d' % (x, y))
        # print("BRG", frame[y][x])
    elif event == cv2.EVENT_RBUTTONDOWN:
        # output YAML for fence
        # grep FENCE video.log | cut -c47- > fence.yaml
        logger.info("FENCE - x: %d", x)
        logger.info("FENCE   y: %d", y)


def main():
    # logging.basicConfig()
    with open("logging.yaml") as f:
        logging.config.dictConfig(yaml.full_load(f))

    logger.info("Starting video control")

    config = jsonobject.fromJson(yaml.full_load(open("config.yaml")))

    # fence = box(580, 350, 1200, 600)
    # print(config.fence)
    fence = Polygon(list(((p.x, p.y) for p in config.fence)))
    aoi = fence.buffer(config.video.aoi_buffer, resolution=1, join_style=2)

    if config.robot.control:
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

    cap = cv2.VideoCapture(config.video.url)

    show = True
    # show = False
    roi_mask = empty_image(cap)
    cv2.fillPoly(roi_mask, [np.array([[int(x), int(y)] for x, y in aoi.exterior.coords])], White)
    if show: cv2.imshow("roi", roi_mask)

    no_frame = 0
    # TODO: could add keyboard commands here beyond quit: record, stop, ...
    while cv2.waitKey(1) & 0xFF != ord('q'):
        ret, frame = cap.read()
        print(ret)
        if not ret:
            logger.warning("No frame %r", ret)
            no_frame += 1
            if no_frame > 10: break
            continue
        no_frame = 0
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

        # Process frame before drawing in it
        contours = diff.process2(frame, roi_mask, show=show)

        draw_exterior(fence, frame, (0, 0, 255), 2)
        draw_exterior(aoi, frame, (0, 0, 0), 2)

        cv2.imshow('frame', frame)
        cv2.setMouseCallback('frame', onMouse)

        if course is None:
            course = frame.copy()
            cv2.imshow('course', course)

        position = None
        # process contours in the order of decreasing area
        contours.sort(key=cv2.contourArea)
        contours.reverse()
        for c in contours:
            M = cv2.moments(c)
            try:
                A, cX, cY = area_and_centroid(M)
            except:
                continue

            # For shapely - has float x,y, cv2 expects ints for drawing
            p = Point(cX, cY)

            if A > config.video.area_max:
                logger.debug("Area too large %d %d %.2f", cX, cY, A)
                continue

            if A < config.video.area_min:
                logger.debug("Area too small %d %d %.2f", cX, cY, A)
                # logger.debug("Invalid area %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
                # cv2.circle(frame, (cX, cY), 15, Red, 2)
                break

            if not aoi.contains(p):
                logger.debug("Invalid point %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
                cv2.circle(frame, (cX, cY), 15, Blue, 2)
                continue

            logger.debug("Valid area %d %d %.2f %r %.2f", cX, cY, A, aoi.contains(p), fence.exterior.distance(p))
            position = (cX, cY)
            cv2.circle(frame, position, 25, Black, 2)

        cv2.imshow("frame", frame)

        if position is not None:

            logger.info("Point %d %d %r %.2f %.2f", cX, cY, fence.contains(p), fence.exterior.distance(p), A)

            if last_point:
                cv2.line(course, last_point, position, (10, 10, 10), 2)
            last_point = position
            cv2.imshow("course", course)

            t = time.time()

            # Buffer zone outside the fence: will run avoidance maneouvre.
            # Get into an outside state, if staying outside too long, then abort
            if not fence.contains(p):

                if not outside:
                    logger.info("EXIT")
                    outside = True
                    if config.robot.control:
                        robot.avoid(config.robot.speed, config.robot.turnRate)
                    # Only tries it once! So don't have to wait for it to complete.
                    # Will only continue when if the avoid ends up inside
                    continue

                # TODO: check if avoidance is fininshed...
                continue

                assert outside
                if fence.exterior.distance(p) < config.robot.max_outside_distance:
                    if config.robot.control:
                        robot.avoid(config.robot.speed, config.robot.turnRate)
                    # Only tries it once! So don't have to wait for it to complete.
                    # Will only continue when if the avoid ends up inside
                    continue

                continue

            # inside fence
            if outside:
                logger.info("ENTER")
                time_into_inside = t
                # robot.send(robot.Speed(speed))
                outside = False

        # process below also when no position

        if not outside and config.robot.inside_timeout and t > time_into_inside + config.robot.inside_timeout:
            logger.warning("INSIDE timeout")
            if config.robot.control:
                robot.send(robot.Stop)
            continue

        if robot.battery_level is not None:
            # TODO: only print this now and then - when received in robot
            # logger.info("Battery level %.3f", robot.battery_level)
            if robot.battery_level < config.robot.battery_cutoff:
                logger.warning("Battery level %.3f low - stopping", robot.battery_level)
                if config.robot.control:
                    robot.send(robot.Stop)
                continue

        # logger.debug("Avoidance %r", avoid_complete.isSet())
        if avoid_complete.isSet():
            # this is a problem if just reversing out of fence, immediately goes forward.
            # but only from manual GUI control?
            # robot.send(robot.Speed(config.robot.speed))
            # else:
            # don't mess up the avoidance
            if config.robot.control:
                robot.send(robot.Heartbeat)
        else:
            logger.info("In avoidance")

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        

if __name__ == '__main__':
    main()