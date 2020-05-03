import sys
import cv2
import numpy as np
import logging
from colors import *

logger = logging.getLogger("diff")

background = None
roi_mask = None

"""
If robot is present in the first frame, then absdiff() gives big signal where robot was at frame 0 after it has
started moving. Update background continuously to adapt to slowly. 
"""


backSub = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=32, detectShadows=False)
# backSub = cv2.createBackgroundSubtractorKNN(history=200, dist2Threshold=3000, detectShadows=False)


def process2(frame, roi=None, show=False):

    blur = cv2.GaussianBlur(frame, (15, 15), 0)
    # Apparently 5x faster, only for rectangular ROI:
    # blur2 = np.zeros(img.shape, np.uint8)
    # blur2[y:y + h, x:x + w] = blur[y:y + h, x:x + w]
    if roi is not None:
        blur = cv2.bitwise_and(blur, roi)
    if show: cv2.imshow("Blur", blur)

    thresh_frame = backSub.apply(blur)
    thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)
    if show:
        cv2.imshow("thresh", thresh_frame)
    try:
        contours, hierarchy = cv2.findContours(thresh_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # cv2.CHAIN_APPROX_SIMPLE)
    except:
        logger.debug("No contours")
        contours = []
    return contours


def process(frame, roi=None, show=False):

    global background, roi_mask
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (15, 15), 0)
    if show:
        cv2.imshow("Blurred", gray)

    if roi is not None:
        gray = cv2.bitwise_and(gray, roi)

    if background is None:
        background = gray

    diff_frame = cv2.absdiff(background, gray)
    if show:
        cv2.imshow("Difference", diff_frame)

    # TODO: only threshold on  positive difference, e.g. ignore removal of white objects: robot, flowers cut, ...
    thresh_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
    thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)
    if show:
        cv2.imshow("Threshold Frame", thresh_frame)

    # Finding contour of moving object
    try:
        contours, hierarchy = cv2.findContours(thresh_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.CHAIN_APPROX_NONE)
        return contours
    except:
        logger.debug("No contours")
        return None


def motion(cap):
    #video.set(2, 1540)
    #show = False
    show = True

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # print(w, h, fps, n_frames)

    roi = np.zeros(shape=[h, w, 3], dtype=np.uint8)
    cv2.rectangle(roi, (1450, 50), (2400, 800), White, cv2.FILLED)
    if show: cv2.imshow("roi", roi)

    i = 0
    while cap.isOpened():
        check, frame = cap.read()
        if frame is None:
            break
        i += 1

        # logger.debug("frame %d", i)
        #if i < 1540: continue

        # contours = process(frame, show=True)
        contours = process2(frame, roi, show=show)

        contours.sort(key=cv2.contourArea)
        contours.reverse()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 300:
                #logger.debug("Too small %.1f", area)
                break

            M = cv2.moments(contour)
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            center = (center_x, center_y)
            x, y, w, h = cv2.boundingRect(contour)
            if show:
                cv2.rectangle(frame, (x, y), (x + w, y + h), Green, 3)
                cv2.circle(frame, center, 15, Black, 2)
            logger.info("POINT %d %d %d", i, center_x, center_y)
            break

        if show:
            cv2.imshow("frame", frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

        # if i > 60: time.sleep(4)

    video.release()
    cv2.destroyAllWindows()


def main():
    logging.basicConfig(level=logging.DEBUG)
    cap = cv2.VideoCapture('recording/2019-09-10-184511.avi')
    motion(cap)


if __name__ == '__main__':
    main()
