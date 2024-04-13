import cv2
import numpy as np
import time


low_mask = np.array([100, 25, 110])
high_mask = np.array([140, 160, 220])


mode2 = False

def set_mode2():
    global mode2, low_mask, high_mask
    mode2 = True
    low_mask = np.array([85, 60, 120])
    high_mask = np.array([115, 255, 255])


def frame_preprocess(frame: cv2.Mat) -> cv2.Mat:
    frame = cv2.resize(frame, (1000, 1000), interpolation=cv2.INTER_AREA)
    if(mode2):
        frame = frame[0:500, 500:1000]
    else:
        frame = frame[0:300, 200:800]
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
        #print("direction err 1")
        return 0, debug_img
    
    #cv2.imshow("post", post_img)

    valid_obj = filt_by_arc_length(contours)
    #print('arc length valid: ', len(valid_obj))

    if len(valid_obj) >= 2 and mode2:
        return -1, post_img

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


sample_dur = 1
last_sample_start = time.time()
samples = [0, 0, 0]
filt_threshold = 0.5
def dir_filt(dir):
    global last_sample_start, samples
    if time.time() - last_sample_start > sample_dur:
        sample_cnt = max(samples[0] + samples[1] + samples[2], 1)
        max_cnt = 0
        max_cnt_idx = 0
        for idx in range(0, 3):
            if samples[idx] > max_cnt:
                max_cnt = samples[idx]
                max_cnt_idx = idx
        
        last_sample_start = time.time()
        samples = [0, 0, 0]
        if max_cnt / sample_cnt > filt_threshold:
            return max_cnt_idx - 1
        else:
            return 0
    else:
        samples[dir + 1] += 1
        return 0







#### main start ####
if __name__ == "__main__":
    car_test = True
    #set_mode2()
    
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
        filted_dir = dir_filt(dir)
        if filted_dir != 0:
            print("Filted: ", filted_dir)

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        elif key == 32:
            continue


    cap.release()
    cv2.destroyAllWindows()