import sys
import cv2
import time
import numpy as np
import logging
import logging.config
import yaml
import math
from colors import *


def motion(video):
    from datetime import datetime

    # Assigning our static_back to None
    static_back = None

    # List when any moving object appear
    motion_list = [None, None]

    while video.isOpened():

        check, frame = video.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (15, 15), 0)

        # Displaying image in gray_scale
        cv2.imshow("Blurred", gray)

        # In first iteration we assign static_back to our first frame
        if static_back is None:
            static_back = gray

        diff_frame = cv2.absdiff(static_back, gray)
        cv2.imshow("Difference", diff_frame)

        # TODO: only threshold on  positive difference, e.g. ignore removal of white objects: robot, flowers cut, ...
        thresh_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
        thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)
        cv2.imshow("Threshold Frame", thresh_frame)

        # Finding contour of moving object
        try:
            contours, hierarchy = cv2.findContours(thresh_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if 0:
                _, contours, _ = cv2.findContours(thresh_frame, #.copy(),
                                                  cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 100:
                    # print("Too small", area)
                    continue

                M = cv2.moments(contour)
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                center = (center_x, center_y)
                (x, y, w, h) = cv2.boundingRect(contour)
                # making green rectangle around the moving object
                cv2.rectangle(frame, (x, y), (x + w, y + h), Green, 3)
                cv2.circle(frame, center, 15, Green, 2)

        except:
            print("No contours")

        # Color frame with contour of motion of object
        cv2.imshow("Frame", frame)

        sys.stdout.flush()
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()


def main():
    logging.basicConfig()
    cap = cv2.VideoCapture('recording/2019-09-10-184511.avi')
    motion(cap)


if __name__ == '__main__':
    main()