import matplotlib.pyplot as plt
import cv2
import numpy as np
import math as m
import copy

box = np.array([[4000, 4000], [6500, 6500], [6500, 4000], [4000, 6500]])
last_box = box.copy()
center = 5000
half_edge_length = 5000
enter_point = [center, center]

fig = plt.figure(figsize=(6, 6))
ax = fig.subplots(1,1)


def move(dist_xy, deg):
    global box, last_box
    last_box = copy.deepcopy(box)
    #print('lb-1: ', last_box)
    sum_xy = np.array([0.0, 0.0])
    for point in box:
        point[0] += dist_xy[0]
        point[1] += dist_xy[1]
        sum_xy += point
    sum_xy = np.divide(sum_xy, 4)
    #print(sum_xy)
    rad = m.radians(deg)
    new_box = []
    for point in box:
        delta = point - sum_xy
        rot_mat = np.array([[m.cos(rad), m.sin(rad)], [-m.sin(rad), m.cos(rad)]])
        new_box.append(sum_xy + delta @ rot_mat)
    box = new_box
    #print('lb-2: ', last_box)


def select_enter_point():
    global enter_point
    min_dist = 1e10
    for point in box:
        dist = np.linalg.norm(np.array([center, center]) - point)
        if dist < min_dist:
            min_dist = dist
            enter_point = point


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
        for point in predict_box:
            delta = point - last_center
            test_box.append(last_center + delta @ rot_mat)
        test_box = np.array(test_box)
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







def update_plt():
    global enter_point
    enter_point = tracing_point(enter_point)
    #print(get_move())

    ax.cla()
    ax.axis([center - half_edge_length, center + half_edge_length, center - half_edge_length, center + half_edge_length])

    last_box_point = np.transpose(last_box)
    #ax.scatter(last_box_point[0], last_box_point[1], color="blue")
    box_point = np.transpose(box)
    ax.scatter(box_point[0], box_point[1], color="green")
    draw_enter_point = np.transpose(enter_point)
    ax.scatter(draw_enter_point[0], draw_enter_point[1], color="black")

    ax.scatter([center], [center], color="red")
    plt.draw()
    plt.pause(1)




# main
if __name__ == "__main__":
    print('A')
    select_enter_point()
    update_plt()

    print('B')
    move([1000, 0], 0)
    update_plt()

    print('C')
    move([0, -2000], 0)
    update_plt()
    
    print('D')
    move([0, 0], 40)
    update_plt()
    
    print('E')
    move([0, 0], 30)
    update_plt()
    
    print('F')
    move([0, 0], 40)
    update_plt()


    while 1:
        plt.pause(1)
        pass


