import cv2
import numpy as np
import time


low_mask = np.array([92, 100, 180])
high_mask = np.array([163, 255, 255])

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
    


def filt_by_tree(contours, hierarchy) -> np.array:
    hierarchy = hierarchy[0]
    contours = np.array(contours, dtype=object)
    #print("file in", hierarchy)
    del_list = []
    for idx in range(len(hierarchy)):
        hierarchy_ele = hierarchy[idx]
        if hierarchy_ele[3] == -1:
            del_list.append(idx)

    #print(del_list)
    hierarchy_filted = np.delete(hierarchy, del_list, 0)
    contours_filted = np.delete(contours, del_list, 0)
    return contours_filted, hierarchy_filted



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
        valid = True
        for exist_obj in valid_objs:
            if abs(arc_len - exist_obj['arc_len']) < 30:
                valid = False
                break
        if not valid:
            continue
        valid_objs.append({'cont':contour, 'arc_len':arc_len})
    return valid_objs



def select_smallest(valid_obj) -> dict:
    smallest_obj = {}
    #contours_img = orignal_img.copy()
    for obj in valid_obj:
        #cv2.waitKey(0)
        if smallest_obj == {}:
            smallest_obj = obj
        elif smallest_obj['arc_len'] > obj['arc_len']:
            smallest_obj = obj
    return smallest_obj



def filt_by_points(obj_list, poly_img) -> list:
    valid_obj = []
    for obj in obj_list:
        approx = cv2.approxPolyDP(obj['cont'], 10, True)
        cv2.polylines(poly_img, [approx], True, (0, 0, 255), 2)
        hull = cv2.convexHull(approx)
        cv2.polylines(poly_img, [hull], True, (0, 255, 0), 2)
        #print("approx: ", approx, "\nhull: ", hull)
        obj['approx'] = approx
        obj['hull'] = hull

        #print("point cnt approx/hull: ", len(approx), len(hull))

        if len(approx) >= 5 and len(hull) >= 4:
            valid_obj.append(obj)
    return valid_obj









#### main function ####
def direction_detect(frame: cv2.UMat):
    orignal_img, post_img = frame_preprocess(frame)
    debug_img = orignal_img.copy()

    contours, hierarchy = cv2.findContours(post_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    try:
        #print("raw count: ", len(hierarchy[0]))
        contours, hierarchy = filt_by_tree(contours, hierarchy)
        #print("tree valid: ", len(hierarchy))
    except:
        print("direction err 1")
        return 0, debug_img
    
    #cv2.imshow("orignal", orignal_img)
    #cv2.imshow("test0.5", blur_img)
    #cv2.imshow("test1", canny_img)
    #cv2.imshow("test1.5", dilate_img)
    #cv2.imshow("post", post_img)

    valid_obj = filt_by_arc_length(contours)
    #print('arc length valid: ', len(valid_obj))

    valid_obj = filt_by_points(valid_obj, debug_img)
    #print('point count valid: ', len(valid_obj))

    valid_obj = select_smallest(valid_obj)
    if(valid_obj == {}):
        #print("direction err")
        return 0, debug_img

    approx = valid_obj['approx']
    hull = valid_obj['hull']

    highest = np.array([0, 10000])
    lowest = np.array([0, -10000])
    leftest = np.array([10000, 0])
    rightest = np.array([-10000, 0])
    for point in hull:
        point = np.array(point[0])
        if point[1] < highest[1]:
            highest = point
        elif  point[1] > lowest[1]:
            lowest = point

        if point[0] < leftest[0]:
            leftest = point
        elif point[0] > rightest[0]:
            rightest = point

        cv2.circle(debug_img, (point[0], point[1]), radius=3, color=(0, 255, 255), thickness=3)

    cv2.circle(debug_img, (highest[0], highest[1]), radius=5, color=(0, 0, 0), thickness=3)
    cv2.circle(debug_img, (lowest[0], lowest[1]), radius=5, color=(100, 100, 100), thickness=3)

    if highest[0] < lowest[0]:
        #print("left")
        cv2.putText(debug_img, "left", tuple(leftest), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 255), 2, cv2.LINE_AA)
        dir = -1
    elif highest[0] > lowest[0]:
        #print("right")
        cv2.putText(debug_img, "right", tuple(rightest), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 255), 2, cv2.LINE_AA)
        dir = 1
    else:
        #print("unknown")
        dir = 0

    #cv2.imshow("poly", debug_img)

    return dir, debug_img







#### main start ####
if __name__ == "__main__":

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




    while(True):
        ret, frame = cap.read()
        dir, debug_img = direction_detect(frame)
        cv2.imshow("debug", debug_img)

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        elif key == 32:
            continue


    cap.release()
    cv2.destroyAllWindows()