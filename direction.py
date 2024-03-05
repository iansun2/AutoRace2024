import cv2
import numpy as np
import time


car_test = True
if car_test:
    cap = cv2.VideoCapture("/dev/video0")
    if not cap.isOpened():
        print("camera err")
        exit()
    cap.set(3,5000)
    cap.set(4,5000)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,1)
else:
    cap = cv2.VideoCapture(2+cv2.CAP_DSHOW)



low_mask = np.array([92, 100, 73])
high_mask = np.array([163, 255, 255])


while(True):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (500, 500), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img = cv2.inRange(hsv, low_mask, high_mask)
    kernel_size = 5
    blur_img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 10
    canny_img = cv2.Canny(blur_img, low_threshold, high_threshold)
    #contours, hierarchy = cv2.findContours(canny_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    kernel = np.ones((2,2),np.uint8)
    dilate_img = cv2.dilate(canny_img, kernel, iterations=2)
    erode_img = cv2.erode(dilate_img, kernel, iterations=1)
    #gradient_img = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)
    #gradient_img = cv2.morphologyEx(canny_img, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy_all = cv2.findContours(erode_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    print(hierarchy_all[0])
    print("raw count: ", len(hierarchy_all[0]))

    hierarchy_all = hierarchy_all[0]
    del_list = []
    for idx in range(len(hierarchy_all)):
        hierarchy = hierarchy_all[idx]
        if hierarchy[2] == -1 and hierarchy[3] == -1:
            del_list.append(idx)

    #print(del_list)
    hierarchy_all = np.delete(hierarchy_all, del_list, 0)
    contours = np.delete(contours, del_list, 0)
    
    #print(contours)
    print(hierarchy_all)
    print("filt cont: ", len(hierarchy_all))
    
    
    #img = cv2.cvtColor(hsv, cv2.COLOR_HSV2GRAY)
    cv2.imshow("raw", frame)
    #cv2.imshow("test0.5", blur_img)
    #cv2.imshow("test1", canny_img)
    #cv2.imshow("test1.5", dilate_img)
    #cv2.imshow("test2", erode_img)

    valid_obj = []

    for contour in contours:
        #contours_img = cv2.drawContours(frame.copy(), contour, -1, (0, 255, 0), 2) 
        #cv2.imshow("contours", contours_img)
        #area = cv2.contourArea(contour)
        #print(area)
        arc_len = cv2.arcLength(contour, True)
        #print("arc len: ", arc_len)
        #cv2.waitKey(0)
        if arc_len < 100:
            continue
        valid = True
        for exist_obj in valid_obj:
            if abs(arc_len - exist_obj['arc_len']) < 30:
                valid = False
                break
        if not valid:
            continue
        valid_obj.append({'cont':contour, 'arc_len':arc_len})
        
    print('valid cont cnt: ', len(valid_obj))
        
    smallest_obj = None
    contours_img = frame.copy()
    for obj in valid_obj:
        cv2.drawContours(contours_img, obj['cont'], -1, (0, 255, 0), 2) 
        cv2.imshow("contours", contours_img)
        cv2.waitKey(0)
        if smallest_obj == None:
            smallest_obj = obj
        elif smallest_obj['arc_len'] > obj['arc_len']:
            smallest_obj = obj
    #cv2.imshow("contours", contours_img)

    poly_img = frame.copy()
    for cont in valid_obj:
        approx = cv2.approxPolyDP(cont['cont'], 10, True)
        cv2.polylines(poly_img, [approx], True, (0, 0, 255), 2)
        hull = cv2.convexHull(approx)
        ##hull = cv2.convexHull(approx, returnPoints=False)
        print("approx: ", approx, "\nhull: ", hull)
        cv2.polylines(poly_img, [hull], True, (0, 255, 0), 2)

        print("point cnt approx/hull: ", len(approx), len(hull))

        if len(hull) >= 6 and len(approx) >= 8:
            highest = np.array([0, 10000])
            lowest = np.array([0, -10000])
            leftest = np.array([10000, 0])
            rightest = np.array([-10000, 0])
            for point in hull:
                #print("tp:", point)
                point = np.array(point[0])
                if point[1] < highest[1]:
                    highest = point
                elif  point[1] > lowest[1]:
                    lowest = point

                if point[0] < leftest[0]:
                    leftest = point
                elif point[0] > rightest[0]:
                    rightest = point

                cv2.circle(poly_img, (point[0], point[1]), radius=3, color=(0, 255, 255), thickness=3)
                #cv2.imshow("poly", poly_img)
                #cv2.waitKey(0)

            cv2.circle(poly_img, (highest[0], highest[1]), radius=5, color=(0, 0, 0), thickness=3)
            cv2.circle(poly_img, (lowest[0], lowest[1]), radius=5, color=(100, 100, 100), thickness=3)

            if highest[0] < lowest[0]:
                print("left")
                cv2.putText(poly_img, "left", tuple(leftest), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 255), 2, cv2.LINE_AA)
            elif highest[0] > lowest[0]:
                print("right")
                cv2.putText(poly_img, "right", tuple(rightest), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 255), 2, cv2.LINE_AA)
            else:
                print("unknown")


            # print("slope pos/neg cnt: ", pos_slope, neg_slope)
            # M = cv2.moments(hull)
            # if M["m00"] != 0:#由於除數不能為0所以一定要先設判斷式才不會出錯
            #     cx = int(M["m10"] / M["m00"])#找出中心的x座標
            #     cy = int(M["m01"] / M["m00"])#找出中心的y座標
            # if(pos_slope < neg_slope):
            #     print("right")
            #     cv2.putText(poly_img, "right", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 255), 2, cv2.LINE_AA)
            # elif(pos_slope > neg_slope):
            #     print("left")
            #     cv2.putText(poly_img, "left", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 255), 2, cv2.LINE_AA)
            # else:
            #     print("unknown")

    #cv2.imshow("poly", poly_img)

    key = cv2.waitKey(100) & 0xFF
    if key == ord('q'):
        break
    elif key == 32:
        continue


cap.release()
cv2.destroyAllWindows()