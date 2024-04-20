import cv2
import numpy as np
import time
import motor_controller as mctrl
import atexit
import direction as dd
import trace_line as tl
import lidar as ld
import fence as fc
import camera as cam


### Variable ###
# Stage
# 0: 紅綠燈, 1: 左右路口, 2: 避障
# 3: 停車,   4: 柵欄,    5: 黑箱
stage = 4




# init motor
motor = mctrl.init_motor()
# init lidar
lidar = ld.Lidar()



def exit_handler():
    motor.setSpeed(0, 0)
    #lidar.stop()
    #lidar.disconnect()

atexit.register(exit_handler)





# 看紅綠燈
def HoughCircles():
    #紅綠燈遮罩
    low_G = np.array([65,110,110])
    up_G = np.array([90,255,255])

    # low_G = np.array([160,35,180])
    # up_G = np.array([180,140,255])

    # 回傳值(0=沒看到綠燈,1=有看到綠燈)
    look_green = 0
    # 讀取圖片並轉HSV
    ret, img = cam.get_camera()
    #img = cv2.resize(img, (500, 500))
    img = img[0:150, 400:550]
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
        pass

        
    #顯示新圖像
    cv2.imshow('mask', mask_G)
    cv2.imshow('grad', gradient)
    cv2.imshow('final', final_img)
    cv2.waitKey(1)

    return look_green



last_print = 0

def get_trace(trace_mode, sl_dist, sl_kp, tl_kp) -> int:
    global last_print
    trace_config = [sl_dist[0], sl_dist[1], sl_kp[0], sl_kp[1], tl_kp]
    ret, tl_frame = cam.get_camera()
    #ret, tl_frame = cap.read()
    L_min, R_min, tl_debug_img = tl.get_trace_value(tl_frame)
    trace = tl.trace_by_mode(trace_mode, L_min, R_min, trace_config)
    # if time.time() - last_print > 1:
    #     cv2.line(tl_debug_img, (320, 360), (320 - trace, 280), color=(255, 100, 200), thickness=3)
    cv2.imshow('TraceLine', tl_debug_img)
    #     last_print = time.time()
    key = cv2.waitKey(2) & 0xFF
    return trace


def set_motor(trace, lidar, speed):
    global motor
    target = (trace + lidar) * 0.5
    #print("trace / lidar /target: ", trace, ' / ', lidar, ' / ', target)
    try:
        #print("motor: ", speed - target, " / ", speed + target)
        motor.setSpeed(speed - target, speed + target)
        pass
    except:
        print('[error]motor')
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

cam.wait_camera_avail()

# test test
# while 1:
#     trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.5) # two line 2
#     wait_camera()


time.sleep(3)
run_start_time = time.time()
print('run start')



########[等待紅綠燈]########
if stage == 0:
    print('[Stage] start stage 0: ', time.time() - run_start_time)
    while True:
        look_green = HoughCircles()
        #time.sleep(0.2)
        if look_green == 1:
            print('pass')
            #time.sleep(7)
            break
            pass
    motor.setSpeed(100, 100)
    print('[Stage] end stage 0: ', time.time() - run_start_time)
    stage = 1



########[左右岔路]########
if stage == 1:
    print('[Stage] start stage 1: ', time.time() - run_start_time)
    # go to fork
    start_time = time.time()
    while(not tl.fork_flag):
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.5) # two line 2
        set_motor(trace=trace, lidar=0, speed=250)
        #tl.fork_flag = False

    print('[Info] stage 1 <fork>: ', time.time() - run_start_time)
    # wait camera stable
    set_motor(trace=0, lidar=0, speed=0) # stop motor
    time.sleep(1) # wait 1 sec to stable
    # look sign direction
    img_cnt = 0
    dir_samples = [0, 0, 0] # left none right
    while img_cnt < 10: # 10 samples
        img_cnt += 1
        ret, dd_frame = cam.get_camera()
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
    #max_cnt_idx = 0 # force #############################################
    # most none:
    if max_cnt_idx == 1:
        print('[Error] stage 1 dir None')
        while 1:
            pass
    # most left:
    elif max_cnt_idx == 0:
        print('[Info] stage 1 dir Left')
        motor.goDist(100, 180)
        motor.goRotate(-40, 100)
        motor.setSpeed(200, 200)
        go_dir_time = time.time()
        while time.time() - go_dir_time < 18: # trace left 12 sec
            trace = get_trace(trace_mode=-1, sl_dist=(200, 0), sl_kp=(3.2, 0), tl_kp=0) # left line
            set_motor(trace=trace, lidar=0, speed=250)
            
    # most right:
    elif max_cnt_idx == 2:
        print('[Info] stage 1 dir Right')
        motor.goDist(100, 180)
        motor.goRotate(45, 100)
        motor.setSpeed(200, 200)
        go_dir_time = time.time()
        while time.time() - go_dir_time < 18: # trace right 12 sec
            trace = get_trace(trace_mode=1, sl_dist=(0, 210), sl_kp=(0, 3.2), tl_kp=0) # right line
            set_motor(trace=trace, lidar=0, speed=250)
            
    # two line trace to stage 2
    print('[Info] stage 1 <two line>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.5) # two line
        set_motor(trace=trace, lidar=0, speed=250)
        closest = lidar.get_closest_filt(270, 360, False)
        if closest[0] < 300:
            #set_motor(trace=0, lidar=0, speed=0) # stop motor
            break
        
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
        
    print('[Stage] end stage 3: ', time.time() - run_start_time)
    stage = 4



########[柵欄]########
if stage == 4:
    print('[Stage] start stage 4: ', time.time() - run_start_time)
    # go to fence 
    print('[Info] stage 4 <fence>: ', time.time() - run_start_time)
    go_fence_start = time.time()
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.8) # two line
        set_motor(trace=trace, lidar=0, speed=250)
        #closest = lidar.get_closest_filt(265, 360, False)
        #if closest[0] < 450:
        if time.time() - go_fence_start > 12:
            break
        
    # fence down detect
    print('[Info] stage 4 <fence down detect>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1) # two line
        set_motor(trace=trace, lidar=0, speed=50)
        ret, fc_frame = cam.get_camera()
        fence, fc_debug_img = fc.fence_detect(fc_frame)
        filted_fc = fc.fence_filt(fence)
        cv2.imshow('Fence', fc_debug_img)
        key = cv2.waitKey(2) & 0xFF
        if filted_fc == -1:
            motor.setSpeed(20, 20)
            break
        
    # fence up detect
    print('[Info] stage 4 <fence up detect>: ', time.time() - run_start_time)
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=0.5) # two line
        set_motor(trace=trace, lidar=0, speed=10)
        ret, fc_frame = cam.get_camera()
        fence, fc_debug_img = fc.fence_detect(fc_frame)
        filted_fc = fc.fence_filt(fence)
        cv2.imshow('Fence', fc_debug_img)
        key = cv2.waitKey(2) & 0xFF
        if filted_fc != -1:
            break
        
    # final trace:
    print('[Info] stage 4 <final>: ', time.time() - run_start_time)
    final_start = time.time()
    while 1:
        trace = get_trace(trace_mode=0, sl_dist=(0, 0), sl_kp=(0, 0), tl_kp=1.5) # two line
        set_motor(trace=trace, lidar=0, speed=200)
        closest = lidar.get_closest_filt(30, 90, False)
        if time.time() - final_start > 10:
            break
        


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







cv2.destroyAllWindows()
motor.setSpeed(0, 0)
