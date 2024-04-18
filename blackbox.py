import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math as m
import lidar as ld
import copy 


fig = plt.figure(figsize=(6, 6))
ax = fig.subplots(1,1)


center = 5000
half_edge_length = 2000
area_ref = 4e6
area_tol = 1e6


box = [[center, center], [center, center - half_edge_length * 2], [center - half_edge_length * 2, center], [center - half_edge_length * 2, center - half_edge_length * 2]]
last_box = copy.deepcopy(box)
enter_point = [center, center]
exit_point = [center - half_edge_length * 2, center - half_edge_length * 2]


def get_map(lidar : ld.Lidar):
    global ax, fig, box, last_box

    data = lidar.get_angle_data() # [deg, dist] * N
    #print(np.array(view))
    points = []
    for point in data:
        #print(data)
        dist = point[1]
        deg = point[0]
        if dist < 2500:
            rad = m.radians(deg + 90)
            x = dist * m.cos(rad) + center
            y = center - dist * m.sin(rad)
            points.append([int(x), int(y)])

    ax.cla()
    ax.axis([center - half_edge_length, center + half_edge_length, center - half_edge_length, center + half_edge_length])

    points = np.array(points)
    raw_points = np.transpose(points)

    points = cv2.approxPolyDP(points, 50, True)
    approx_points = np.transpose(points)
    
    _box = cv2.minAreaRect(points)
    _box = np.intp(cv2.boxPoints(_box))
    area = cv2.contourArea(_box)
    
    if area > area_ref + area_tol or area < area_ref - area_tol:
        #print('area err: ', area)
        return -1
    else:
        #print('area ok: ', area)
        last_box = box.copy()
        box = _box.copy()
        enter_point = tracing_point(enter_point)
        exit_point = tracing_point(exit_point)
        pass

    
    polygon1 = Polygon(box, True)
    ax.add_patch(polygon1)
    ax.scatter([center], [center], color="blue")
    ax.scatter([enter_point[0]], [enter_point[1]], color="green")
    ax.scatter([exit_point[0]], [exit_point[1]], color="red")

    ax.scatter(approx_points[0], approx_points[1], color="orange")

    plt.draw()
    plt.pause(1)

    return 0



def select_enter_exit_point():
    global enter_point, exit_point
    # enter point
    min_dist = 1e10
    for point in box:
        dist = np.linalg.norm(np.array([center, center]) - point)
        if dist < min_dist:
            min_dist = dist
            enter_point = point
    # exit point
    max_dist = 0
    for point in box:
        dist = np.linalg.norm(enter_point - point)
        if dist > max_dist:
            max_dist = dist
            exit_point = point



def get_move():
    current_center = np.array([0.0, 0.0])
    for point in box:
        current_center += point
    current_center = np.divide(current_center, 4)
    last_center = np.array([0.0, 0.0])
    for point in last_box:
        last_center += point
    last_center = np.divide(last_center, 4)
    #print('lc: ', last_center)
    #print('cc: ', current_center)
    delta_xy = current_center - last_center
    predict_box = last_box + delta_xy

    error_min = 1e10
    best_deg = 0
    for deg in range(-45, 46, 5):
        rad = m.radians(deg)
        rot_mat = np.array([[m.cos(rad), m.sin(rad)], [-m.sin(rad), m.cos(rad)]])
        test_box = []
        # get test box
        for point in predict_box:
            delta = point - last_center
            test_box.append(last_center + delta @ rot_mat)
        test_box = np.array(test_box)
        # calcuate error between test box and real box
        error_box = test_box - box
        error = 0
        for point in error_box:
            error += np.linalg.norm(point)
        if error < error_min:
            best_deg = deg
            error_min = error

    return [delta_xy, best_deg]



def tracing_point(last_point):
    current_center = np.array([0.0, 0.0])
    for point in box:
        current_center += point
    current_center = np.divide(current_center, 4)

    move = get_move()
    predict_point = last_point + move[0]
    delta = predict_point - current_center
    rad = m.radians(move[1])
    rot_mat = np.array([[m.cos(rad), m.sin(rad)], [-m.sin(rad), m.cos(rad)]])
    predict_point = current_center + delta @ rot_mat

    real_point = np.array([0.0, 0.0])
    min_dist = 1e10
    for point in box:
        dist = np.linalg.norm(predict_point - point)
        if dist < min_dist:
            min_dist = dist
            real_point = point

    return real_point



# main
if __name__ == "__main__":
    #freeze_support()
    lidar = ld.Lidar()
    plt.ion()
    plt.show()
    while get_map(lidar):
        pass
    select_enter_exit_point()
    while 1:
        get_map(lidar)
        

