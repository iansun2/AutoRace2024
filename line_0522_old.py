import cv2
import matplotlib.pyplot as plt 
import numpy as np
import time
import serial

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
cap = cv2.VideoCapture(0)
cap.set(3,5000)
cap.set(4,5000)
cap.set(cv2.CAP_PROP_BRIGHTNESS,1)

# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('/home/iclab/autorace/line/output3.avi', fourcc, 30.0, (640, 360))

# 左右線HSV遮色閥值
L_H_low = 8
L_S_low = 8
L_V_low = 240
L_H_high = 36
L_S_high = 80
L_V_high = 255

R_H_low = 0
R_S_low = 0
R_V_low = 230
R_H_high = 0
R_S_high = 0
R_V_high = 255

# 右線遮罩
lower_R = np.array([R_H_low,R_S_low,R_V_low])
upper_R = np.array([R_H_high,R_S_high,R_V_high])

# 左線遮罩
lower_L = np.array([L_H_low,L_S_low,L_V_low])
upper_L = np.array([L_H_high,L_S_high,L_V_high])

#紅綠燈遮罩
low_G = np.array([35,90,90])
up_G = np.array([85,255,255])


# 採樣間距
W_sampling_1 = 325
W_sampling_2 = 290
W_sampling_3 = 255
W_sampling_4 = 220


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




while 1:
    ret, img = cap.read()
    cv2.imshow('final',img)
    cv2.waitKey(1)