import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import camera as cam

debug = False


low_mask = np.array([0, 140, 70])
high_mask = np.array([15, 255, 255])

low_mask2 = np.array([160, 140, 70])
high_mask2 = np.array([180, 255, 255])


def frame_preprocess(frame: cv2.Mat) -> cv2.Mat:
    #frame = cv2.resize(frame, (500, 500), interpolation=cv2.INTER_AREA)
    frame = frame[0:150, 200:320]
    frame = cv2.resize(frame, (300, 240), interpolation=cv2.INTER_LINEAR)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, low_mask, high_mask)
    mask2 = cv2.inRange(hsv, low_mask2, high_mask2)
    img = mask1 + mask2
    if debug:
        cv2.imshow('raw', frame)
        cv2.imshow('mask', img)
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
        if arc_len < 50 or arc_len > 200:
            continue
        valid_objs.append({'cont':contour, 'arc_len':arc_len})
    return valid_objs



def filt_by_points(obj_list, poly_img) -> list:
    valid_objs = []
    for obj in obj_list:
        approx = cv2.approxPolyDP(obj['cont'], 7, True)
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
    #print("after arc len filt: ", len(valid_objs))
    valid_objs = filt_by_points(valid_objs, debug_img)
    #print("after point filt: ", len(valid_objs))
    valid_objs = filt_by_position(valid_objs, debug_img)
    #print("after position filt: ", len(valid_objs))
    #cv2.imshow("debug2", orignal_img)

    # 1: up, 0: none, -1: down
    status = 0

    if len(valid_objs) > 2:
        data = []
        for obj in valid_objs:
            data.append(obj['center'])
        data = np.array(data)
        data = data.transpose()
        # plt.clf()
        # plt.axis([0,500,500,0])
        # plt.scatter(data[0], data[1], color="red")

        linear_model = np.polyfit(data[0], data[1], 1)
        linear_model_fn = np.poly1d(linear_model)
        print("linear: ", linear_model_fn)
        # x_s = np.arange(0, 500)
        # y_s = linear_model_fn(x_s)
        # plt.plot(x_s, y_s, color="green")

        # plt.title("Scatter Plot of the data")
        # plt.xlabel("X")
        # plt.ylabel("Y")
        m = linear_model_fn.c[0]
        cv2.putText(debug_img, 'coef: ' + f"{m:.2f}", (150, 250), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 255), 1, cv2.LINE_AA)
        #plt.draw()
        #plt.pause(0.001)
        if abs(m) < 1:
            status = -1
        else:
            status = 1
            
        cv2.putText(debug_img, 'status: ' + str(status), (150, 300), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 255), 1, cv2.LINE_AA)

    return status, debug_img.copy()



sample_dur = 1
last_sample_start = time.time()
samples = [0, 0, 0]
filt_threshold = 0.5
def fence_filt(fence):
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
        samples[fence + 1] += 1
        return 0





# main
if __name__ == "__main__":
    debug = True
    cam.wait_camera_avail()

    plt.ion()
    plt.show()

    while(True):
        ret, frame = cam.get_camera()
        dir, debug_img = fence_detect(frame)
        cv2.imshow("debug", debug_img)
        

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        elif key == 32:
            continue

    cv2.destroyAllWindows()

