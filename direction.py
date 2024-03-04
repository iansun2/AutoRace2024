import cv2
import numpy as np
import time

cap = cv2.VideoCapture(2+cv2.CAP_DSHOW)


low_mask = np.array([92, 155, 73])
high_mask = np.array([163, 255, 255])


while(True):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (500, 500), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img = cv2.inRange(hsv, low_mask, high_mask)
    kernel_size = 25
    blur_img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 10
    canny_img = cv2.Canny(blur_img, low_threshold, high_threshold)
    #contours, hierarchy = cv2.findContours(canny_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    kernel = np.ones((3,3),np.uint8)
    dilate_img = cv2.dilate(canny_img, kernel, iterations=2)
    erode_img = cv2.erode(dilate_img, kernel, iterations=1)
    #gradient_img = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)
    #gradient_img = cv2.morphologyEx(canny_img, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy = cv2.findContours(erode_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    print("raw cont: ", len(contours))
    
    #img = cv2.cvtColor(hsv, cv2.COLOR_HSV2GRAY)
    cv2.imshow("test1", canny_img)
    cv2.imshow("test1.5", dilate_img)
    cv2.imshow("test2", erode_img)

    valid_contour = []

    for contour in contours:
        #contours_img = cv2.drawContours(frame.copy(), contour, -1, (0, 255, 0), 3) 
        #cv2.imshow("contours", contours_img)
        #area = cv2.contourArea(contour)
        #print(area)
        arc_len = cv2.arcLength(contour, True)
        #print(arc_len)
        #cv2.waitKey(0)
        if arc_len < 500:
            continue
        valid = True
        for exist_contour in valid_contour:
            if abs(arc_len - exist_contour['arc_len']) < 100:
                valid = False
                break
        if not valid:
            continue
        valid_contour.append({'cont':contour, 'arc_len':arc_len})
        
    
    print('cont cnt: ', len(valid_contour))

    poly_img = frame.copy()
    for cont in valid_contour:
        approx = cv2.approxPolyDP(cont['cont'], 8, True)
        cv2.polylines(poly_img, [approx], True, (0, 0, 255), 2)
        hull = cv2.convexHull(approx)
        cv2.polylines(poly_img, [hull], True, (0, 255, 0), 2)
        #print(hull)
        print("line cnt approx/hull: ", len(approx), len(hull))
        #cv2.imshow("poly", poly_img)
        #cv2.waitKey(0)
    cv2.imshow("poly", poly_img)

    key = cv2.waitKey(100) & 0xFF
    if key == ord('q'):
        break
    elif key == 32:
        continue


cap.release()
cv2.destroyAllWindows()