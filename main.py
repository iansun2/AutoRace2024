import cv2
import matplotlib.pyplot as plt 
import numpy as np
import time
import motor_controller as mctrl
import atexit
import math as m
import direction as dd
import trace_line as tl



'''
    序列埠設定
    開啟權限: sudo chmod 666 /dev/ttyACM0
    https://www.itdaan.com/tw/1b422ec424e1c3519f7cee4c2ad05274
'''
#ser=serial.Serial('/dev/ttyACM0',9600,timeout=0.5)
# ser.flushInput()            # 清除輸入緩存區，放棄所有內容
# ser.flushOutput()           # 清除輸出緩衝區，放棄輸出
# ser.open()


# 相機設定
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
	print("camera err")
	exit()
cap.set(3,5000)
cap.set(4,5000)
cap.set(cv2.CAP_PROP_BRIGHTNESS,1)



# trace mode
# -1: left, 0: two line, 1: right
trace_mode = 0
# dist between screen center and line
trace_line_dist = 180
# single line start time
single_line_st = 0



motor = mctrl.init_motor()


def exit_handler():
    motor.setSpeed(0, 0)

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








# 等待紅綠燈
# while True:
#     look_green = HoughCircles()
#     if look_green==1:
#         break



# 正式開始
while True:

    t = time.time()

    # 讀取圖片並轉HSV
    ret, img = cap.read()

    # direction
    dd_frame = img.copy()
    dir, dd_debug_img = dd.direction_detect(dd_frame)

    # trace line
    L_min, R_min = tl.get_trace_value(img)

    # direction to trace mode
    # left
    if dir == -1 and trace_mode == 0:
        single_line_st = time.time()
        trace_mode = -1
    # right
    elif dir == 1 and trace_mode == 0:
        single_line_st = time.time()
        trace_mode = 1
    # release
    elif (time.time() - single_line_st) > 30:
        print("release")
        trace_mode = 0

    print("trace mode: ", trace_mode)


    # trace mode
    # left line
    if trace_mode == -1:
        target_line = int(L_min - 150) * 2
    # right line
    elif trace_mode == 1:
        target_line = int(170 - R_min) * 3
    # two lines
    else:
        target_line = int(L_min-R_min)


    # draw target line
    cv2.line(img, (320, 360), (320 - target_line, 280), color=(255, 100, 200), thickness=3)

    print("target line: ", target_line)

    print("corr: ", target_line)
    target_line *= 0.5
    motor.setSpeed(100 - target_line, 100 + target_line)




    # 輸出原圖&成果
    cv2.imshow("img", img)
    cv2.imshow("dd", dd_debug_img)
    # out.write(img)

    #cv2.imshow("mask_R", mask_R)
    #cv2.imshow("mask_L", mask_L)
    # cv2.waitKey(0)

     

    k=cv2.waitKey(1)
    if k==ord('q'):
        cv2.destroyAllWindows()
        motor.setSpeed(0, 0)
        break



cap.release()
cv2.destroyAllWindows()
motor.setSpeed(0, 0)
