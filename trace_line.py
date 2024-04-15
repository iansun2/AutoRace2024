import cv2
import numpy as np

# flag
fork_flag = False



# 左右線HSV遮色閥值
L_H_low = 0
L_S_low = 0
L_V_low = 220
L_H_high = 36
L_S_high = 80
L_V_high = 255

R_H_low = 0
R_S_low = 0
R_V_low = 235
R_H_high = 180
R_S_high = 20
R_V_high = 255

# 右線遮罩
lower_R = np.array([R_H_low,R_S_low,R_V_low])
upper_R = np.array([R_H_high,R_S_high,R_V_high])

# 左線遮罩
lower_L = np.array([L_H_low,L_S_low,L_V_low])
upper_L = np.array([L_H_high,L_S_high,L_V_high])




# 採樣間距
W_sampling_1 = 325
W_sampling_2 = 290
W_sampling_3 = 255
W_sampling_4 = 220



def get_trace_value(img : cv2.UMat):
    global fork_flag
    # 左右線X值(需重置)
    R_min_300 = 640
    R_min_240 = 640
    R_min_180 = 640
    R_min_140 = 640

    L_min_300 = 0
    L_min_240 = 0
    L_min_180 = 0
    L_min_140 = 0

    # 重設大小、轉HSV
    img = cv2.resize(img,(640,360))
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # 右線遮罩
    mask_R = cv2.inRange(hsv,lower_R,upper_R)
    # 左線遮罩
    mask_L = cv2.inRange(hsv,lower_L,upper_L)


    # Right
    # Canny邊緣運算
    kernel_size = 25
    blur_gray = cv2.GaussianBlur(mask_R,(kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 20
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 閉運算(解緩Canny斷線問題)
    kernel = np.ones((5,5),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    #霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    
    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)<R_min_300:
                    R_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)<R_min_240:
                    R_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_3:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)<R_min_180:
                    R_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_4:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)<R_min_140:
                    R_min_140 = int((x1+x2)/2)
    else:
        pass


    # Left
    # Canny邊緣運算
    kernel_size = 25
    blur_gray = cv2.GaussianBlur(mask_L,(kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 20
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 閉運算(解緩Canny斷線問題)
    kernel = np.ones((5,5),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    #霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)

    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_1:
                # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                if ((x1+x2)/2)>L_min_300:
                    L_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_2:
                # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                if ((x1+x2)/2)>L_min_240:
                    L_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_3:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)>L_min_180:
                    L_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_4:
                # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                if ((x1+x2)/2)>L_min_140:
                    L_min_140 = int((x1+x2)/2)    
    else:
        pass


    # cv2.rectangle(img, (L_min_300, W_sampling_1), (R_min_300, 360), (255,0,0), 0) 
    # cv2.rectangle(img, (L_min_240, W_sampling_2), (R_min_240, W_sampling_1), (0,255,0), 0) 
    # cv2.rectangle(img, (L_min_180, W_sampling_3), (R_min_180, W_sampling_2), (0,0,255), 0) 
    # cv2.rectangle(img, (L_min_140, W_sampling_4), (R_min_140, W_sampling_3), (0,255,255), 0) 

    # Left
    pts = np.array([[L_min_300,(360+W_sampling_1)/2], [L_min_240,(W_sampling_1+W_sampling_2)/2], [L_min_180,(W_sampling_2+W_sampling_3)/2],[L_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)
    for point in pts:
        #print("point: ", point)
        cv2.circle(img, tuple(point[0]), 3, color=(255, 0, 200), thickness=3)

    # Right
    pts = np.array([[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)
    for point in pts:
        #print("point: ", point)
        cv2.circle(img, tuple(point[0]), 3, color=(255, 0, 200), thickness=3)

    # Fork Detect
    print('L:', L_min_300 - L_min_240, L_min_240 - L_min_180, L_min_180 - L_min_140) # screen low -> high
    print('R:', R_min_240 - R_min_300, R_min_180 - R_min_240, R_min_140 - R_min_180)
    if L_min_240 - L_min_180 > 5 and L_min_180 - L_min_140 > 15:
        if R_min_240 - R_min_300 >= 0:
            print('[Debug] Fork')
            fork_flag = True


    L_min = 320-((L_min_300+L_min_240+L_min_180+L_min_140)/4)
    R_min = ((R_min_300+R_min_240+R_min_180+R_min_140)/4)-320
    #print("min L/R: ", L_min, " ", R_min)

    return L_min, R_min, img, mask_L, mask_R



# dist between screen center and line
# config : [single_line_dist_L, single_line_dist_R, single_line_kp_L, single_line_kp_R, two_line_kp]

def trace_by_mode(trace_mode : int, L_min : int, R_min : int, config : list):
    # trace mode
    # left line
    if trace_mode == -1:
        trace = (L_min - config[0]) * config[2]
    # right line
    elif trace_mode == 1:
        trace = (config[1] - R_min) * config[3]
    # two lines
    else:
        trace = (L_min-R_min) * config[4]

    return int(trace)