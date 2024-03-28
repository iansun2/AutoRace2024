import cv2
import numpy as np
import time

low_mask = np.array([0, 100, 100])
high_mask = np.array([20, 255, 255])


def frame_preprocess(frame: cv2.Mat) -> cv2.Mat:
    frame = cv2.resize(frame, (500, 500), interpolation=cv2.INTER_AREA)
    frame = frame[0:500, 100:400]
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
    return frame, erode_img



def filt_by_arc_length(contours) -> list:
    valid_objs = []

    for contour in contours:
        #contours_img = cv2.drawContours(frame.copy(), contour, -1, (0, 255, 0), 2) 
        #cv2.imshow("contours", contours_img)
        arc_len = cv2.arcLength(contour, True)
        #print("arc len: ", arc_len)
        #cv2.waitKey(0)
        if arc_len < 100:
            continue
        valid_objs.append({'cont':contour, 'arc_len':arc_len})
    return valid_objs



def filt_by_points(obj_list, poly_img) -> list:
    valid_objs = []
    for obj in obj_list:
        approx = cv2.approxPolyDP(obj['cont'], 10, True)
        cv2.polylines(poly_img, [approx], True, (0, 255, 0), 2)
        #hull = cv2.convexHull(approx)
        #cv2.polylines(poly_img, [hull], True, (0, 255, 0), 2)
        #print("approx: ", approx, "\nhull: ", hull)
        obj['approx'] = approx
        #obj['hull'] = hull
        #print("point cnt approx/hull: ", len(approx), len(hull))
        if len(approx) <= 5 and len(approx) >= 3 :
            valid_objs.append(obj)
    return valid_objs


def filt_by_position(obj_list, poly_img) -> list:
    valid_objs = []
    for obj in obj_list:
        M = cv2.moments(obj['cont'])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        center = np.array([cX, cY])
        obj['center'] = center
        valid = True
        for exist_obj in valid_objs:
            if np.linalg.norm(center - exist_obj['center']) < 30:
                valid = False
                break
        if not valid:
            continue
        cv2.circle(poly_img, center, 3, (255, 0, 0), -1)
        valid_objs.append(obj)
    return valid_objs



def fence_detect(frame: cv2.UMat):
    orignal_img, post_img = frame_preprocess(frame)
    debug_img = orignal_img.copy()
    #debug_img = post_img.copy()
    contours, hierarchy = cv2.findContours(post_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    valid_objs = filt_by_arc_length(contours)
    print("after arc len filt: ", len(valid_objs))
    valid_objs = filt_by_points(valid_objs, debug_img)
    print("after point filt: ", len(valid_objs))
    valid_objs = filt_by_position(valid_objs, debug_img)
    print("after position filt: ", len(valid_objs))

    return 0, orignal_img.copy()



# main
if __name__ == "__main__":
    car_test = False

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



    while(True):
        ret, frame = cap.read()
        dir, debug_img = fence_detect(frame)
        cv2.imshow("debug", debug_img)

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        elif key == 32:
            continue


    cap.release()
    cv2.destroyAllWindows()

