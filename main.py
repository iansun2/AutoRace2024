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
import copy

#np.seterr(all="ignore")

### Variable ###
# Stage
# 0: 紅綠燈, 1: 左右路口, 2: 避障
# 3: 停車,   4: 柵欄,    5: 黑箱
stage = 4
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
default_trace_config = [200, 190, 4, 2.7, 1.5]
current_trace_config = default_trace_config.copy()
# global timer
timer_timeout_flag = False
timer_start_time = 0
timer_start = False
timer_timeout = 1
# common counter
common_counter = 0

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



def exit_handler():
    motor.setSpeed(0, 0)
    #lidar.stop()
    #lidar.disconnect()

atexit.register(exit_handler)





# Camera
mutex_camera = threading.Lock()
ret = None
img = None
frame_cnt = 0
frame_cnt_start = time.time()

def camera_handler():
    global ret, img, frame_cnt
    _ret, _img = cap.read()
    mutex_camera.acquire()
    frame_cnt += 1
    ret = _ret
    img = _img
    if time.time() - frame_cnt_start > 1:
        print('camera fps: ', frame_cnt)
        frame_cnt_start = time.time()
    mutex_camera.release()
camera_thread = threading.Thread(target=camera_handler)


def get_camera():
    mutex_camera.acquire()
    _ret = copy.deepcopy(ret)
    _img = copy.deepcopy(img)
    mutex_camera.release()
    return _ret, _img


def wait_camera():
    global camera_thread
    camera_thread.join()
    camera_thread.start()




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
    circles= cv2.HoughCircles(gradient,cv2.HOUGH_GRADIENT,1,20,param1=45,param2=20,minRadius=5,maxRadius=80)

    
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
    #cv2.imshow('final', final_img)
    cv2.waitKey(1)

    return look_green



last_print = 0

def get_trace(trace_mode, sl_dist, sl_kp, tl_kp) -> int:
    global last_print
    trace_config = [sl_dist[0], sl_dist[1], sl_kp[0], sl_kp[1], tl_kp]
    tl_frame = img.copy()
    L_min, R_min, tl_debug_img, mask_L, mask_R = tl.get_trace_value(tl_frame)
    trace = tl.trace_by_mode(trace_mode, L_min, R_min, trace_config)
    if time.time() - last_print > 0.2:
        cv2.imshow('TraceLine', tl_debug_img)
        last_print = time.time()
        key = cv2.waitKey(2) & 0xFF
    return trace


def set_motor(trace, lidar, speed):
    global motor
    target = (trace + lidar) * 0.5
    #print("trace / lidar /target: ", trace, ' / ', lidar, ' / ', target)
    try:
        #print("motor: ", current_trace_speed - target, " / ", current_trace_speed + target)
        motor.setSpeed(speed - target, speed + target)
        pass
    except:
        motor = mctrl.init_motor()
        pass



# #test
# while(1):
#     if new_img_flag:
#         ret, img = get_camera()
#         trace = get_trace(trace_mode=0, sl_dist=(200, 190), sl_kp=(4, 2.7), tl_kp=1.5) # two line
#         if tl.fork_flag:
#             print('fork!')
#             tl.fork_flag = False



########[主程式]########

while ret is None or img is None:
    pass
time.sleep(3)
run_start_time = time.time()
print('run start')



########[等待紅綠燈]########
if stage == 0:
    print('[Stage] start stage 0: ', time.time() - run_start_time)
    while True:
        look_green = HoughCircles()
        time.sleep(0.2)
        if look_green == 1:
            print('pass')
            #time.sleep(7)
            break
            pass
    print('[Stage] end stage 0: ', time.time() - run_start_time)
    stage = 1



########[左右岔路]########
if stage == 1:
    print('[Stage] start stage 1: ', time.time() - run_start_time)
    # go to fork
    while(not tl.fork_flag):
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=2) # two line
        set_motor(trace=trace, lidar=0, speed=250)
        wait_camera()

    print('[Info] stage 1 <fork>: ', time.time() - run_start_time)
    # wait camera stable
    set_motor(trace=0, lidar=0, speed=0) # stop motor
    time.sleep(1) # wait 1 sec to stable
    # look sign direction
    img_cnt = 0
    dir_samples = [0, 0, 0] # left none right
    while img_cnt < 10: # 10 samples
        img_cnt += 1
        ret, dd_frame = get_camera()
        dir, dd_debug_img = dd.direction_detect(dd_frame)
        cv2.imshow('Direction', dd_debug_img)
        key = cv2.waitKey(20) & 0xFF
        dir_samples[dir+1] += 1
        wait_camera()
    # get most dir
    max_cnt = 0
    max_cnt_idx = 0
    for idx in range(0, 3):
        if dir_samples[idx] > max_cnt:
            max_cnt = dir_samples[idx]
            max_cnt_idx = idx
    print('[Info] stage 1 <dir>: ', time.time() - run_start_time)
    # most none:
    if max_cnt_idx == 1:
        print('[Error] stage 1 dir None')
        while 1:
            pass
    # most left:
    elif max_cnt_idx == 0:
        print('[Info] stage 1 dir Left')
        motor.goRotate(-30, 100)
        motor.setSpeed(100, 100)
        go_dir_time = time.time()
        while time.time() - go_dir_time < 12: # trace left 12 sec
            trace = get_trace(trace_mode=-1, sl_dist=(200, 0), sl_kp=(3, 0), tl_kp=0) # left line
            set_motor(trace=trace, lidar=0, speed=200)
            wait_camera()
    # most right:
    elif max_cnt_idx == 2:
        print('[Info] stage 1 dir Right')
        motor.goRotate(30, 100)
        motor.setSpeed(100, 100)
        go_dir_time = time.time()
        while time.time() - go_dir_time < 12: # trace right 12 sec
            trace = get_trace(trace_mode=1, sl_dist=(0, 200), sl_kp=(0, 3), tl_kp=0) # right line
            set_motor(trace=trace, lidar=0, speed=200)
            wait_camera()
    # two line trace to stage 2
    print('[Info] stage 1 <two line>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.5) # two line
        set_motor(trace=trace, lidar=0, speed=250)
        closest = lidar.get_closest_filt(270, 360, False)
        if closest[0] < 300:
            #set_motor(trace=0, lidar=0, speed=0) # stop motor
            break
        wait_camera()
    print('[Stage] end stage 1: ', time.time() - run_start_time)
    stage = 2



########[避障]########
if stage == 2:
    print('[Stage] start stage 2: ', time.time() - run_start_time)
    # avoidance
    exit_cnt = 0
    avoidance_start = time.time()
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=2) # two line
        lidar_val = lidar.get_avoidance(fov=180, filt_dist=500, kp=5e-4)
        set_motor(trace=trace, lidar=lidar_val, speed=250)
        exit_lidar_val = lidar.get_closest_filt(90, 360, False)
        #print(exit_lidar_val)
        if exit_lidar_val[0] > 350 and time.time() - avoidance_start > 10: # init lock 10 sec
            print('exit cnt++: ', exit_cnt)
            exit_cnt += 1
        if exit_cnt > 5: # exit need 5 count
            break
        wait_camera()
    print('[Stage] end stage 2: ', time.time() - run_start_time)
    stage = 3

    

########[停車]########
if stage == 3:
    print('[Stage] start stage 3: ', time.time() - run_start_time)
    # single trace until close to 
    print('[Info] stage 3 <get close>: ', time.time() - run_start_time)
    left_trace_start = time.time()
    while 1:
        trace = get_trace(trace_mode=-1, sl_dist=(200, 0), sl_kp=(2.5, 0), tl_kp=0) # left line
        set_motor(trace=trace, lidar=0, speed=200)
        closest = lidar.get_closest_filt(45, 315, True)
        print(closest)
        if closest[0] < 450 and time.time() - left_trace_start > 3: # min exit > 3 sec
            #set_motor(trace=0, lidar=0, speed=0) # stop
            break
        wait_camera()

    # go forward until wall at left or right
    print('[Info] stage 3 <final close>: ', time.time() - run_start_time)
    motor.setSpeed(200, 200)
    while 1:
        closest = lidar.get_closest()
        print(closest)
        deg = closest[1]
        if (deg > 85 and deg < 95) or (deg > 265 and deg < 275):
            #set_motor(trace=0, lidar=0, speed=0) # stop
            break
    # parking
    print('[Info] stage 3 <parking>: ', time.time() - run_start_time)
    closest = lidar.get_closest()
    motor.goDist(100, 200)
    # right full
    if closest[1] < 180:
        park_dist = 250
        park_speed = 150
        print("right full")
        motor.goRotate(90, park_speed)
        motor.goDist(-park_dist, park_speed)
        time.sleep(1)
        print("leave slot")
        motor.goDist(park_dist, park_speed)
        motor.goRotate(90, park_speed)
    # left full
    else:
        park_dist = 250
        park_speed = 150
        print("left full")
        motor.goRotate(-90, park_speed)
        motor.goDist(-park_dist, park_speed)
        time.sleep(1)
        print("leave slot")
        motor.goDist(park_dist, park_speed)
        motor.goRotate(-90, park_speed)
    # go to line
    motor.goDist(150, 150)
    # left trace to stage 4
    left_trace_start = time.time()
    while 1:
        trace = get_trace(trace_mode=-1, sl_dist=(200, 0), sl_kp=(2, 0), tl_kp=0) # left line
        set_motor(trace=trace, lidar=0, speed=150)
        if time.time() - left_trace_start > 10: # trace 5 sec
            break
        wait_camera()
    print('[Stage] end stage 3: ', time.time() - run_start_time)
    stage = 4



########[柵欄]########
if stage == 4:
    print('[Stage] start stage 4: ', time.time() - run_start_time)
    # go to fence 
    print('[Info] stage 4 <fence>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=2) # two line
        set_motor(trace=trace, lidar=0, speed=250)
        closest = lidar.get_closest_filt(265, 360, False)
        if closest[0] < 450:
            break
        wait_camera()
    # fence down detect
    print('[Info] stage 4 <fence down>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1) # two line
        set_motor(trace=trace, lidar=0, speed=50)
        fc_frame = img.copy()
        fence, fc_debug_img = fc.fence_detect(fc_frame)
        filted_fc = fc.fence_filt(fence)
        cv2.imshow('Fence', fc_debug_img)
        key = cv2.waitKey(2) & 0xFF
        if filted_fc == -1:
            motor.setSpeed(10, 10)
            break
        wait_camera()
    # fence up detect
    print('[Info] stage 4 <fence up>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=0.5) # two line
        set_motor(trace=trace, lidar=0, speed=10)
        fc_frame = img.copy()
        fence, fc_debug_img = fc.fence_detect(fc_frame)
        filted_fc = fc.fence_filt(fence)
        cv2.imshow('Fence', fc_debug_img)
        key = cv2.waitKey(2) & 0xFF
        if filted_fc == 1:
            break
        wait_camera()
    # final trace:
    print('[Info] stage 4 <final>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.5) # two line
        set_motor(trace=trace, lidar=0, speed=200)
        closest = lidar.get_closest_filt(30, 90, False)
        if closest[0] < 250:
            break
        wait_camera()


    print('[Stage] end stage 4: ', time.time() - run_start_time)
    stage = 5



########[黑箱]########
if stage == 5:
    print('[Stage] start stage 5: ', time.time() - run_start_time)


    print('[Stage] end stage 5: ', time.time() - run_start_time)
    stage = 6



motor.setSpeed(0, 0)
print('[End]: ', time.time() - run_start_time)
while 1:
    pass







cap.release()
cv2.destroyAllWindows()
motor.setSpeed(0, 0)
