import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0+cv2.CAP_DSHOW)


low_mask = np.array([92, 155, 73])
high_mask = np.array([163, 255, 255])


while(True):
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img = cv2.inRange(hsv, low_mask, high_mask)
    kernel_size = 25
    blur_img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    low_threshold = 10
    high_threshold = 20
    canny_img = cv2.Canny(blur_img, low_threshold, high_threshold)
    #contours, hierarchy = cv2.findContours(canny_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    kernel = np.ones((3,3),np.uint8)
    #gradient_img = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)
    gradient_img = cv2.morphologyEx(canny_img, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy = cv2.findContours(gradient_img.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    print(hierarchy)
    
    #img = cv2.cvtColor(hsv, cv2.COLOR_HSV2GRAY)
    cv2.imshow("test1", canny_img)
    cv2.imshow("test2", gradient_img)

    for contour in contours:
        contours_img = cv2.drawContours(frame.copy(), contour, -1, (0, 255, 0), 3) 
        cv2.imshow("contours", contours_img)
        cv2.waitKey(0)
        

    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        break
    elif key == 32:
        continue


cap.release()
cv2.destroyAllWindows()