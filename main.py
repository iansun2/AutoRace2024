import cv2
import matplotlib.pyplot as plt 
import numpy as np
import time
import motor_controller as mctrl
import atexit
import math as m
import direction as dd
import trace_line as tl
from rplidar import RPLidar
import lidar as ld
import threading


### Variable ###
# Stage
# 0: 紅綠燈, 1: 左右路口, 2: 避障
# 3: 停車,   4: 停車離場, 5: 柵欄
stage = 1
# enable
disable_trace = False
disable_lidar_trace = True
disable_drive = False

# trace mode
# -1: left, 0: two line, 1: right
trace_mode = 0
# single line start time
single_line_st = 0



# 相機設定
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
	print("camera err")
	exit()
cap.set(3,1000)
cap.set(4,1000)
cap.set(cv2.CAP_PROP_BRIGHTNESS,1)





# init motor
motor = mctrl.init_motor()


def exit_handler():
    motor.setSpeed(0, 0)
    #lidar.stop()
    #lidar.disconnect()

atexit.register(exit_handler)



# 看紅綠燈
def HoughCircles():

    # 回傳值(0=沒看到綠燈,1=有看到綠燈)
    look_green = 0

    # 讀取圖片並轉HSV
    ret, img = cap.read()
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    

    # 遮罩
    mask_G = cv2.inRange(hsv,low_G,up_G)
    
    guess_img = cv2.GaussianBlur(mask_G,(25,25), 0)

    #param1的具體實現，用於邊緣檢測    
    canny = cv2.Canny(guess_img, 20, 45)

    kernel = np.ones((2,2),np.uint8)
    gradient = cv2.morphologyEx(canny, cv2.MORPH_GRADIENT, kernel)


    #霍夫變換圓檢測
    circles= cv2.HoughCircles(gradient,cv2.HOUGH_GRADIENT,2,200,param1=45,param2=85,minRadius=1,maxRadius=80)

    
    final_ing = img.copy()
    print('no look green light')
    if (circles) is not None:
        look_green = 1
        #根據檢測到圓的信息，畫出每一個圓
        circles = np.uint16(np.around(circles))
        for circle in circles[0,:]:

            #坐標行列(就是圓心)
            x=int(circle[0])
            y=int(circle[1])
            #半徑
            r=int(circle[2])
            #在原圖用指定顏色圈出圓，參數設定為int所以圈畫存在誤差
            cv2.circle(final_ing,(x,y),r,(255,0,0),2)
            cv2.circle(final_ing,(x,y),5,(0,0,255),3)

        
    #顯示新圖像
    final_ing = cv2.resize(final_ing,(640,360))
    cv2.imshow('final',final_ing)
    cv2.waitKey(1)

    return look_green





# Camera
mutex_camera = threading.Lock()
ret = None
img = None
def camera_handler():
    global ret, img
    while(1):
        _ret, _img = cap.read()
        mutex_camera.acquire()
        ret = _ret
        img = _img
        mutex_camera.release()
camera_thread = threading.Thread(target=camera_handler)
camera_thread.start()

def get_camera():
    mutex_camera.acquire()
    _ret = ret
    _img = img
    mutex_camera.release()
    return _ret, _img




# 等待紅綠燈
# while True:
#     look_green = HoughCircles()
#     if look_green==1:
#         break

while ret is None or img is None:
    pass


print_img_time = time.time() 
# 正式開始
while True:
    t = time.time()

    # 讀取圖片並轉HSV
    ret, img = get_camera()

    # direction process
    dd_frame = img.copy()
    dir, dd_debug_img = dd.direction_detect(dd_frame)
    filted_dir = dd.dir_filt(dir)
    # debug
    # dir = 0

    # trace line process
    tl_frame = img.copy()
    L_min, R_min, tl_debug_img, mask_L, mask_R = tl.get_trace_value(tl_frame)
    # debug
    # L_min = 0
    # R_min = 0
    # tl_debug_img = img.copy()


    # 左右岔路
    if stage == 1:
        # direction to trace mode
        # left
        if filted_dir == -1 and trace_mode == 0:
            single_line_st = time.time()
            trace_mode = -1
        # right
        elif filted_dir == 1 and trace_mode == 0:
            single_line_st = time.time()
            trace_mode = 1
        # release
        elif (time.time() - single_line_st) > 30:
            #print("release")
            trace_mode = 0
            # to stage 2
            stage = 2
            disable_lidar_trace = False
            disable_trace = False
        print("trace mode: ", trace_mode)

    # 避障
    elif stage == 2:
        if filted_dir != 0:
            # to stage 3
            stage = 3
            disable_lidar_trace = True
            disable_trace = False
            trace_mode = -1
    
    # 停車
    elif stage == 3:
        if ld.get_lidar_closest()[0] < 500:
            motor.setSpeed(30, 30)

            closest = ld.get_lidar_closest()
            while closest[1] > 280 or closest[1] < 80 or (closest[1] > 100 and closest[1] < 260):
                print(ld.get_lidar_closest())
                closest = ld.get_lidar_closest()
                pass
            motor.setSpeed(0, 0)
            
            # left full
            if closest < 180:
                motor.goRotate(-90, 30)
                motor.goDist(-50, 30)
                time.sleep(1)
                motor.goDist(50, 30)
                motor.goRotate(-90, 30)
            # right full
            else:
                motor.goRotate(90, 30)
                motor.goDist(-50, 30)
                time.sleep(1)
                motor.goDist(50, 30)
                motor.goRotate(90, 30)
            
            motor.goDist(50, 30)
            # to stage 4
            stage = 4
            disable_trace = False
            disable_lidar_trace = True
            trace_mode = -1
    
    # 離開停車
    elif stage == 4:
        if filted_dir != 0:
            motor.goDist(50, 30)
            # to stage 5
            disable_trace = False
            disable_lidar_trace = True
            trace_mode = 0

        

    # trace
    if not disable_trace:
        # get trace
        trace = tl.trace_by_mode(trace_mode, L_min, R_min)
        # draw trace
        cv2.line(tl_debug_img, (320, 360), (320 - trace, 280), color=(255, 100, 200), thickness=3)
    else:
        trace = 0

    # lidar
    if not disable_lidar_trace:
        # get lidar
        lidar_target = ld.get_lidar_target()
        # draw lidar_target
        cv2.line(tl_debug_img, (320, 360), (320 - lidar_target, 280), color=(100, 100, 200), thickness=3)
    else:
        lidar_target = 0


    # drive
    if not disable_drive:
        target = trace + lidar_target
        # draw target
        cv2.line(tl_debug_img, (320, 360), (320 - target, 280), color=(200, 200, 200), thickness=3)

        print("trace / lidar /target: ", trace, ' / ', lidar_target, ' / ', target)
        target *= 0.5

        try:
            #motor.setSpeed(200 - target, 200 + target)
            pass
        except:
            motor = mctrl.init_motor()
            pass




    # 輸出原圖&成果
    if time.time() - print_img_time > 0.2:
        cv2.imshow("img", tl_debug_img)
        #cv2.imshow("dd", dd_debug_img)

        #cv2.imshow("mask_R", mask_R)
        #cv2.imshow("mask_L", mask_L)
        # cv2.waitKey(0)
        print_img_time = time.time()


     

    k=cv2.waitKey(1)
    if k==ord('q'):
        cv2.destroyAllWindows()
        motor.setSpeed(0, 0)
        break

    #print(time.time())



cap.release()
cv2.destroyAllWindows()
motor.setSpeed(0, 0)
