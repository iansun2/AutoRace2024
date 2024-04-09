import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math as m
import lidar as ld


center = 5000

def get_map():
    view = ld.get_lidar_view()
    points = []
    for data in view:
        dist = data[1]
        rad = m.radians(data[0] + 90)
        x = dist * m.cos(rad) + center
        y = center - dist * m.sin(rad)
        points.append(int(x), int(y))

    points = cv2.convexHull(points)
    box = cv2.minAreaRect(points)
    box = np.int0(cv2.boxPoints(box))

    plt.clf()
    plt.axis([0,10000,0,10000])

    plt_point = np.transpose(np.array(points))

    polygon1 = Polygon(points)
    fig, ax = plt.subplots(1,1)
    ax.add_patch(polygon1)
    plt.scatter([0], [0], color="red")

    plt.draw()
    plt.pause(0.001)



# main
if __name__ == "__main__":
    plt.ion()
    plt.show()
