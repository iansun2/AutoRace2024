import cv2
import numpy as np
import time

cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
	print("camera err")
	exit()
cap.set(3,800)
cap.set(4,600)
cap.set(cv2.CAP_PROP_BRIGHTNESS,1)

cnt = 0
last_print = time.time()

while 1:
    _ret, _img = cap.read()
    #print(_img.shape)
    cnt += 1
    if time.time() - last_print > 1:
        print('cnt: ', cnt)
        cnt = 0
        last_print = time.time()