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
import fence as fc

#np.seterr(all="ignore")

### Variable ###
# Stage
# 0: 紅綠燈, 1: 左右路口, 2: 避障
# 3: 停車,   4: 停車離場, 5: 看柵欄
# 6: 等柵欄, 
stage = 1
# enable
disable_trace = False
disable_lidar_trace = True
disable_drive = False

# trace mode
# -1: left, 0: two line, 1: right
trace_mode = 0
# trace speed
default_trace_speed = 100
current_trace_speed = default_trace_speed
# single line start time
single_line_st = 0
# trace config [single_line_dist_L, single_line_dist_R, single_line_kp_L, single_line_kp_R, two_line_kp]
default_trace_config = [200, 200, 5, 4.5, 2]
current_trace_config = default_trace_config.copy()
# global timer
timer_timeout_flag = False
timer_start_time = 0
timer_start = False
timer_timeout = 1

def timer_handle():
    global timer_timeout_flag, timer_start, timer_start_time, timer_timeout
    #print('timer handle', time.time() - timer_start_time)
    if timer_start and time.time() - timer_start_time > timer_timeout:
        print('end timer')
        timer_timeout_flag = True
        timer_start = False



def start_timer(t):
    global timer_timeout_flag, timer_start_time, timer_start, timer_timeout
    print('start timer: ', t)
    timer_timeout_flag = False
    timer_start = True
    timer_start_time = time.time()
    timer_timeout = t




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
# init lidar
lidar = ld.Lidar()

def exit_handler():
    motor.setSpeed(0, 0)
    #lidar.stop()
    #lidar.disconnect()

atexit.register(exit_handler)





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




# 看紅綠燈
def HoughCircles():
    #紅綠燈遮罩
    low_G = np.array([35,30,200])
    up_G = np.array([60,255,255])

    # 回傳值(0=沒看到綠燈,1=有看到綠燈)
    look_green = 0
    # 讀取圖片並轉HSV
    ret, img = get_camera()
    #img = cv2.resize(img, (500, 500))
    img = img[0:500, 500:1000]
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # 遮罩
    mask_G = cv2.inRange(hsv,low_G,up_G)
    guess_img = cv2.GaussianBlur(mask_G,(25,25), 0)

    #param1的具體實現，用於邊緣檢測    
    canny = cv2.Canny(guess_img, 20, 45)
    kernel = np.ones((2,2),np.uint8)
    gradient = cv2.morphologyEx(canny, cv2.MORPH_GRADIENT, kernel)
    #霍夫變換圓檢測
    circles= cv2.HoughCircles(gradient,cv2.HOUGH_GRADIENT,1,20,param1=45,param2=20,minRadius=1,maxRadius=80)

    
    final_img = img.copy()
    if (circles) is not None:
        print('green: ', len(circles))
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
            cv2.circle(final_img,(x,y),r,(255,0,0),2)
            cv2.circle(final_img,(x,y),5,(0,0,255),3)

    else:
        print('no look green light')

        
    #顯示新圖像
    cv2.imshow('final', img)
    cv2.waitKey(1)

    return look_green






while ret is None or img is None:
    pass

start_time = time.time()
current_trace_speed = 300


#等待紅綠燈
# while True:
#     look_green = HoughCircles()
#     time.sleep(0.2)
#     if look_green==1:
#         print('pass')
#         break
#         pass


print_img_time = time.time() 
# 正式開始
while True:
    timer_handle()
    t = time.time()

    # 讀取圖片並轉HSV
    ret, img = get_camera()

    # direction process
    dd_frame = img.copy()
    dir, dd_debug_img = dd.direction_detect(dd_frame)
    filted_dir = dd.dir_filt(dir)
    #print(filted_dir)
    # debug
    # dir = 0

    # trace line process
    tl_frame = img.copy()
    L_min, R_min, tl_debug_img, mask_L, mask_R = tl.get_trace_value(tl_frame)
    # debug
    # L_min = 0
    # R_min = 0
    # tl_debug_img = img.copy()

    # fence process
    fc_frame = img.copy()
    fence, fc_debug_img = fc.fence_detect(fc_frame)
    filted_fc = fc.fence_filt(fence)


    # 左右岔路
    if stage == 1:
        # direction to trace mode
        # left
        if filted_dir == -1 and trace_mode == 0:
            single_line_st = time.time()
            trace_mode = -1
            print("stage 1 to left")
        # right
        elif filted_dir == 1 and trace_mode == 0:
            single_line_st = time.time()
            trace_mode = 1
            print("stage 1 to right")
        # release
        elif (time.time() - single_line_st) > 18 and single_line_st:
            #print("release")
            trace_mode = 0
            # to stage 2
            stage = 2
            disable_lidar_trace = False
            disable_trace = False
            current_trace_speed = 100
            current_trace_config = [170, 200, 3.5, 4.5, 1.2]
            dd.set_mode2()
            print("go to stage 2")
            #start_timer(20)
        #print('stage1: ', trace_mode, filted_dir)

    # 避障
    elif stage == 2:
        # 看到標誌指向任何一邊
        if filted_dir != 0:
            # to stage 3
            stage = 3
            disable_lidar_trace = True
            disable_trace = False
            trace_mode = -1
            current_trace_config = [190, 170, 2, 3, 1.5]
            motor.setSpeed(0, 0)
            time.sleep(1)
            print("go to stage 3")
            start_timer(10) # 遠離避障

    
    # 停車
    elif stage == 3:
        #print(lidar.get_closest()[0], timer_timeout_flag)
        if timer_timeout_flag and lidar.get_closest()[0] < 450:
            timer_timeout_flag = False
            print("enter parking mode: ", lidar.get_closest())
            motor.setSpeed(60, 60)

            closest = lidar.get_closest()
            while closest[1] > 280 or closest[1] < 80 or (closest[1] > 100 and closest[1] < 260):
                #print(ld.get_lidar_closest())
                closest = lidar.get_closest()
                pass
            print("stop 1")
            motor.goDist(100, 50)
            motor.setSpeed(0, 0)
            
            # right full
            if closest[1] < 180:
                park_dist = 250
                park_speed = 50
                print("right full")
                time.sleep(1)
                motor.goRotate(90, 30)
                motor.goDist(-park_dist, park_speed)
                time.sleep(1)
                print("leave slot")
                motor.goDist(park_dist, park_speed)
                motor.goRotate(100, 30)
            # left full
            else:
                park_dist = 250
                park_speed = 50
                print("left full")
                time.sleep(1)
                motor.goRotate(-90, 30)
                motor.goDist(-park_dist, park_speed)
                time.sleep(1)
                print("leave slot")
                motor.goDist(park_dist, park_speed)
                motor.goRotate(-100, 30)
            
            motor.goDist(250, 50)
            # go to stage 4
            print("go to stage 4")
            stage = 4
            disable_trace = False
            disable_lidar_trace = True
            current_trace_config = default_trace_config.copy()
            trace_mode = -1
            # start timer
            start_timer(30)
    
    # 離開停車
    elif stage == 4:
        # timeout flag
        if timer_timeout_flag:
            timer_timeout_flag = False
            # go to stage 5
            print("go to stage 5")
            stage = 5
            disable_trace = False
            disable_lidar_trace = True
            trace_mode = 0
    
    # 看柵欄
    elif stage == 5:
        print("stage5 filted_fc: ", filted_fc)
        # fence down
        if filted_fc == -1:
            # go to stage 6
            print("go to stage 6")
            stage = 6
            disable_trace = False
            disable_lidar_trace = True
            trace_mode = 0
            current_trace_speed = 20

    # 等柵欄
    elif stage == 6:
        print("stage6 filted_fc: ", filted_fc)
        # fence up
        if filted_fc == 1:
            # go to stage 7
            print("go to stage 7")
            stage = 7
            disable_trace = False
            disable_lidar_trace = True
            trace_mode = 0
            current_trace_speed = default_trace_speed


        

    # trace
    if not disable_trace:
        # get trace
        trace = tl.trace_by_mode(trace_mode, L_min, R_min, current_trace_config)
        # draw trace
        cv2.line(tl_debug_img, (320, 360), (320 - trace, 280), color=(255, 100, 200), thickness=3)
    else:
        trace = 0

    # lidar
    if not disable_lidar_trace:
        # get lidar
        lidar_target = lidar.get_avoidance(config=[180, 500, 15e-4])
        # draw lidar_target
        cv2.line(tl_debug_img, (320, 360), (320 - lidar_target, 280), color=(100, 100, 200), thickness=3)
    else:
        lidar_target = 0


    # drive
    if not disable_drive:
        target = trace + lidar_target
        # draw target
        cv2.line(tl_debug_img, (320, 360), (320 - target, 280), color=(200, 200, 200), thickness=3)

        #print("trace / lidar /target: ", trace, ' / ', lidar_target, ' / ', target)
        target *= 0.5
        #print("motor: ", current_trace_speed - target, " / ", current_trace_speed + target)

        try:
            if time.time() - start_time > 5:
                motor.setSpeed(current_trace_speed - target, current_trace_speed + target)
                pass
        except:
            motor = mctrl.init_motor()
            pass


    # 輸出原圖&成果
    if time.time() - print_img_time > 0.2:
        #cv2.imshow("img", tl_debug_img)
        cv2.imshow("dd", dd_debug_img)

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
