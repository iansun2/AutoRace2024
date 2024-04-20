import cv2
import time
import threading
import copy


# 相機設定
#cap = cv2.VideoCapture("/dev/video0", apiPreference=cv2.CAP_V4L2)
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
	print("camera err")
	exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # 3
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # 4
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual expos
#cap.set(cv2.CAP_PROP_BRIGHTNESS,-3)
cap.set(cv2.CAP_PROP_EXPOSURE, 110)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


# Camera
mutex_camera = threading.Lock()
ret = None
img = None
frame_cnt = 0
frame_cnt_start = time.time()

def camera_handler():
    global ret, img, frame_cnt, frame_cnt_start
    while 1:
        _ret, _img = cap.read()
        mutex_camera.acquire()
        frame_cnt += 1
        ret = _ret
        img = _img
        if time.time() - frame_cnt_start > 1:
            print('camera fps: ', frame_cnt)
            frame_cnt = 0
            frame_cnt_start = time.time()
        mutex_camera.release()
camera_thread = threading.Thread(target=camera_handler)
camera_thread.start()


def get_camera():
    mutex_camera.acquire()
    _ret = copy.deepcopy(ret)
    _img = copy.deepcopy(img)
    mutex_camera.release()
    return _ret, _img


def wait_camera_avail():
    while 1:
        ret, frame = get_camera()
        if frame is not None:
            break



if __name__ == "__main__":
    # _frame_cnt = 0
    # _frame_cnt_start = time.time()
    # while 1:
    #     _ret, _img = cap.read()
    #     _frame_cnt += 1
    #     if time.time() - _frame_cnt_start > 1:
    #         print(_frame_cnt)
    #         _frame_cnt = 0
    #         _frame_cnt_start = time.time()


    wait_camera_avail()
    print('init success')
    #camera_thread.join()
    while 1:
        ret, frame = get_camera()
        #print(frame.shape)
        cv2.imshow('raw', frame)
        key = cv2.waitKey(2) & 0xFF