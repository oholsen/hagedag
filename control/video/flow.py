import numpy as np
import cv2 as cv
import cv2
import time
import logging
import logging.config
import yaml
import math


def lucas_kanade(cap):
    n = 0
    for i in range(60):
        ret, frame1 = cap.read()
        n += 1

    feature_params = dict(maxCorners=100,
                          qualityLevel=0.3,
                          minDistance=7,
                          blockSize=7)
    # Parameters for lucas kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))
    # Take first frame and find corners in it
    ret, old_frame = cap.read()

    roi = np.zeros_like(old_frame)
    roi = cv2.cvtColor(roi, cv2.CV_8UC1) # TODO: reshape
    cv2.rectangle(roi, (1500, 300), (2300, 800), 255, cv2.FILLED)
    cv2.imshow("roi", roi)
    cv2.imshow("first", old_frame)
    #cv2.fillPoly()

    old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

    print(roi.shape)
    print(old_gray.shape)

    p0 = cv.goodFeaturesToTrack(old_gray, mask=roi, **feature_params)
    # Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)

    while True:
        print(n, flush=True)
        ret, frame = cap.read()
        n += 1
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # calculate optical flow
        p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # Select good points
        good_new = p1[st == 1]
        good_old = p0[st == 1]
        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv.line(mask, (a, b), (c, d), color[i].tolist(), 2)
            frame = cv.circle(frame, (a, b), 5, color[i].tolist(), -1)
        img = cv.add(frame, mask)
        cv.imshow('frame', img)
        k = cv.waitKey(30) & 0xff
        if k == 27:
            break
        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)


def dense(cap):
    n = 0
    ret, frame1 = cap.read()
    n += 1
    prvs = cv.cvtColor(frame1,cv.COLOR_BGR2GRAY)
    hsv = np.zeros_like(frame1)
    hsv[...,1] = 255
    while True:
        print(n)
        ret, frame2 = cap.read()
        n += 1
        next = cv.cvtColor(frame2,cv.COLOR_BGR2GRAY)
        flow = cv.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag, ang = cv.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv.normalize(mag,None,0,255,cv.NORM_MINMAX)
        bgr = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
        cv.imshow('frame2', bgr)
        k = cv.waitKey(30) & 0xff
        if k == 27:
            break
        elif k == ord('s'):
            cv.imwrite('opticalfb.png', frame2)
            cv.imwrite('opticalhsv.png', bgr)
        prvs = next


def test(cap):
    while cap.isOpened():
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    logging.basicConfig()
    cap = cv2.VideoCapture('recording/2019-09-10-184511.avi')
    #dense(cap)
    lucas_kanade(cap)

if __name__ == '__main__':
    main()