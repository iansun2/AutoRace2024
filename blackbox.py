import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math as m
import lidar as ld
from multiprocessing import freeze_support


fig = plt.figure(figsize=(6, 6))
ax = fig.subplots(1,1)


center = 5000
half_edge_length = 2000
area_ref = 4e6
area_tol = 1e6


def get_map():
    global ax, fig

    view = ld.get_lidar_view()
    #print(np.array(view))
    points = []
    for deg in range(0, 360):
        #print(data)
        dist = view[deg]
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
    

    box = cv2.minAreaRect(points)
    box = np.intp(cv2.boxPoints(box))
    area = cv2.contourArea(box)
    
    if area > area_ref + area_tol or area < area_ref - area_tol:
        #print('area err: ', area)
        return
    else:
        #print('area ok: ', area)
        pass

    
    polygon1 = Polygon(box, True)
    ax.add_patch(polygon1)
    ax.scatter([center], [center], color="red")

    ax.scatter(approx_points[0], approx_points[1], color="green")

    plt.draw()
    plt.pause(1)



# main
if __name__ == "__main__":
    #freeze_support()
    ld.start_lidar()
    plt.ion()
    plt.show()
    while 1:
        get_map()

