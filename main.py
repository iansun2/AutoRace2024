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
# 3: 停車,   4: 停車離場, 5: 看柵欄
# 6: 等柵欄, 
stage = 3
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
lidar = ld.Lidar()

def exit_handler():
    motor.setSpeed(0, 0)
    #lidar.stop()
    #lidar.disconnect()

atexit.register(exit_handler)



last_print = 0

# Camera
mutex_camera = threading.Lock()
ret = None
img = None
new_img_flag = False
def camera_handler():
    global ret, img, new_img_flag
    while(1):
        _ret, _img = cap.read()
        mutex_camera.acquire()
        ret = _ret
        img = _img
        new_img_flag = True
        mutex_camera.release()
camera_thread = threading.Thread(target=camera_handler)
camera_thread.start()

def get_camera():
    mutex_camera.acquire()
    _ret = copy.deepcopy(ret)
    _img = copy.deepcopy(img)
    mutex_camera.release()
    return _ret, _img

def is_new_img():
    global new_img_flag
    mutex_camera.acquire()
    value = new_img_flag
    new_img_flag = False
    mutex_camera.release()
    return value




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




def get_trace(trace_mode, sl_dist, sl_kp, tl_kp) -> int:
    global last_print
    trace_config = [sl_dist[0], sl_dist[1], sl_kp[0], sl_kp[1], tl_kp]
    tl_frame = img.copy()
    L_min, R_min, tl_debug_img, mask_L, mask_R = tl.get_trace_value(tl_frame)
    trace = tl.trace_by_mode(trace_mode, L_min, R_min, trace_config)
    if time.time() - last_print > 0.2:
        cv2.imshow('TraceLine', tl_debug_img)
        last_print = time.time()
        key = cv2.waitKey(20) & 0xFF
    return trace


def set_motor(trace, lidar, speed):
    global motor
    target = (trace + lidar) * 0.5
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
time.sleep(5)
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
        if is_new_img():
            trace = get_trace(trace_mode=0, sl_dist=(200, 190), sl_kp=(4, 2.7), tl_kp=2.3) # two line
            set_motor(trace=trace, lidar=0, speed=300)
    print('[Info] stage 1 <fork>: ', time.time() - run_start_time)
    # wait camera stable
    set_motor(trace=0, lidar=0, speed=0) # stop motor
    time.sleep(1) # wait 1 sec to stable
    # look sign direction
    img_cnt = 0
    dir_samples = [0, 0, 0] # left none right
    while img_cnt < 20: # 20 samples
        if is_new_img():
            img_cnt += 1
            dd_frame = img.copy()
            dir, dd_debug_img = dd.direction_detect(dd_frame)
            cv2.imshow('Direction', dd_debug_img)
            key = cv2.waitKey(20) & 0xFF
            dir_samples[dir+1] += 1
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
        while time.time() - go_dir_time < 10: # trace left 10 sec
            if is_new_img():
                trace = get_trace(trace_mode=-1, sl_dist=(200, 190), sl_kp=(4, 2.7), tl_kp=1.5) # left line
                set_motor(trace=trace, lidar=0, speed=200)
    # most right:
    elif max_cnt_idx == 2:
        print('[Info] stage 1 dir Right')
        motor.goRotate(30, 100)
        motor.setSpeed(100, 100)
        go_dir_time = time.time()
        while time.time() - go_dir_time < 10: # trace right 10 sec
            if is_new_img():
                trace = get_trace(trace_mode=1, sl_dist=(200, 190), sl_kp=(4, 2.7), tl_kp=1.5) # right line
                set_motor(trace=trace, lidar=0, speed=200)
    # two line trace to stage 2
    print('[Info] stage 1 <two line>: ', time.time() - run_start_time)
    while 1:
        if is_new_img():
            trace = get_trace(trace_mode=0, sl_dist=(200, 190), sl_kp=(4, 2.7), tl_kp=1.5) # two line
            set_motor(trace=trace, lidar=0, speed=200)
        dist = lidar.get_closest()
        if dist < 300:
            set_motor(trace=0, lidar=0, speed=0) # stop motor
            break
    print('[Stage] end stage 1: ', time.time() - run_start_time)
    stage = 2



########[避障]########
if stage == 2:
    print('[Stage] start stage 2: ', time.time() - run_start_time)
    # avoidance
    trace = 0
    exit_cnt = 0
    while 1:
        if is_new_img():
            trace = get_trace(trace_mode=0, sl_dist=(200, 190), sl_kp=(4, 2.7), tl_kp=1.5) # two line
        lidar_val = lidar.get_avoidance(fov=180, filt_dist=500, kp=15e-4)
        set_motor(trace=trace, lidar=0, speed=200)
        exit_lidar_val = lidar.get_closest_filt(90, 360, False)[0]
        if exit_lidar_val > 350:
            exit_cnt += 1
        if exit_cnt > 3: # exit need 3 count
            break
    print('[Stage] end stage 2: ', time.time() - run_start_time)
    stage = 3

    

########[停車]########
if stage == 3:
    print('[Stage] start stage 3: ', time.time() - run_start_time)
    # single trace until close to 
    print('[Info] stage 3 <get close>: ', time.time() - run_start_time)
    left_trace_start = time.time()
    while 1:
        if is_new_img():
            trace = get_trace(trace_mode=-1, sl_dist=(200, 190), sl_kp=(2.2, 2), tl_kp=1.5) # left line
            set_motor(trace=trace, lidar=0, speed=200)
        closest = lidar.get_closest_filt(45, 315, True)
        print(closest)
        if closest[0] < 450 and time.time() - left_trace_start > 5: # min exit > 5 sec
            set_motor(trace=0, lidar=0, speed=0) # stop
            break

    # go forward until wall at left or right
    print('[Info] stage 3 <final close>: ', time.time() - run_start_time)
    motor.setSpeed(150, 150)
    while 1:
        closest = lidar.get_closest()
        print(closest)
        deg = closest[1]
        if (deg > 85 and deg < 95) or (deg > 265 and deg < 275):
            set_motor(trace=0, lidar=0, speed=0) # stop
            break
    # parking
    print('[Info] stage 3 <parking>: ', time.time() - run_start_time)
    closest = lidar.get_closest()
    motor.goDist(100, 150)
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
        if is_new_img():
            trace = get_trace(trace_mode=-1, sl_dist=(200, 190), sl_kp=(2, 2), tl_kp=1.5) # left line
            set_motor(trace=trace, lidar=0, speed=150)
        if time.time() - left_trace_start > 10: # trace 5 sec
            break
    print('[Stage] end stage 3: ', time.time() - run_start_time)
    stage = 4

motor.setSpeed(0, 0)
print('[End]: ', time.time() - run_start_time)
while 1:
    pass










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
        if not timer_timeout_flag and not timer_start and filted_dir != 0:
            print('stop an wait')
            motor.setSpeed(0, 0)
            motor.goDist(-50, 100)
            disable_drive = True
            start_timer(2)
        
        if timer_timeout_flag:
            print('static dir mode')
            #filted_dir = -1 # for 0413 race, -------[NEED REMOVE]--------
            # direction to trace mode
            # left
            if filted_dir == -1 and trace_mode == 0:
                motor.goRotate(-30, 100)
                disable_drive = False
                single_line_st = time.time()
                trace_mode = -1
                print("stage 1 to left")
            # right
            elif filted_dir == 1 and trace_mode == 0:
                motor.goRotate(30, 100)
                disable_drive = False
                single_line_st = time.time()
                trace_mode = 1
                print("stage 1 to right")
            # release
            elif (time.time() - single_line_st) > 15 and single_line_st:
                #print("release")
                # to stage 2
                print("go to stage 2")
                stage = 2
                trace_mode = 0
                disable_lidar_trace = False
                disable_trace = False
                current_trace_speed = 100
                current_trace_config = [170, 200, 3.5, 4.5, 1.2]
                common_counter = 0
                start_timer(30) # 進入避障
        #print('stage1: ', trace_mode, filted_dir)

    # 避障
    elif stage == 2:
        # 看到標誌指向任何一邊
        print('stage 2:', timer_timeout_flag, lidar.get_closest_filt(225, 260)[0], common_counter)
        if common_counter < 3 and timer_timeout_flag and lidar.get_closest_filt(225, 260)[0] > 350:
            common_counter += 1
        if common_counter >= 3 and timer_timeout_flag:
            # to stage 3
            stage = 3
            common_counter = 0
            timer_timeout_flag = False
            disable_lidar_trace = True
            disable_trace = False
            trace_mode = -1
            current_trace_config = [190, 170, 2, 3, 1.5]
            motor.setSpeed(0, 0)
            #time.sleep(1)
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
                motor.goRotate(90, 30)
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
                motor.goRotate(-90, 30)
            
            motor.goDist(150, 50)
            # go to stage 4
            print("go to stage 4")
            stage = 4
            disable_trace = False
            disable_lidar_trace = True
            current_trace_config = [200, 200, 2.7, 4.5, 1.2]
            trace_mode = -1
            # start timer
            start_timer(15)
    
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
            current_trace_speed = 150
            current_trace_config = [170, 200, 2.7, 4.5, 1.5]
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
                #print("motor: ", current_trace_speed - target, " / ", current_trace_speed + target)
                motor.setSpeed(current_trace_speed - target, current_trace_speed + target)
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
