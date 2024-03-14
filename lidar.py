#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time as t
import cv2
from PIL import Image
import threading

PORT_NAME = 'COM9'
DMAX = 400
IMIN = 0
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    #scan_data = [scan[0], scan[1], scan[2], scan[3]]
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    print(t.time())
    return line,

def run():
    lidar = RPLidar(PORT_NAME)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    iterator = lidar.iter_scans()
    #iterator = lidar.iter_measures()
    ani = animation.FuncAnimation(fig, update_line,
        fargs=(iterator, line), interval=50)
    plt.show()
    lidar.stop()
    lidar.disconnect()




lidar = RPLidar(PORT_NAME)
iterator = lidar.iter_scans()

data_size = 400
lidar_list = np.array([[True, 0, 0]] * data_size)
diff_lidar_list = np.array([0] * data_size)

def data_collect():
    current_deg = 0
    idx = 0
    fov = 90
    l_edge = fov / 2
    r_edge = 360 - fov / 2
    filt_dist = 300

    while True:
        err = 0
        scan = next(iterator)
        for data in scan:
            deg = data[1]
            dist = data[2]
            if dist < filt_dist:
                if deg >= r_edge:
                    err += (deg - r_edge) * (dist - filt_dist)
                elif deg < l_edge:
                    err += (deg - l_edge) * (dist - filt_dist)

        print("error: ", err * 5e-3)



#line = None

def show(num, line, line2):
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in lidar_list])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in lidar_list])
    line.set_array(intens)

    # offsets = np.array([(np.radians(meas[0]), meas[1]) for meas in diff_lidar_list])
    # line.set_offsets(offsets)
    # intens = np.array([meas[0] for meas in diff_lidar_list])
    # line.set_array(intens)
    
    return line, line2



def debug():
    st = t.time()
    while 1:
        if(t.time() - st > 1):
            print("A: ", lidar_list)
            print("B: ", diff_lidar_list)
            st = t.time()



if __name__ == '__main__':
    data_thread = threading.Thread(target=data_collect)
    data_thread.start()

    #debug_thread = threading.Thread(target=debug)
    #debug_thread.start()

    # fig = plt.figure()
    # ax = plt.subplot(111, projection='polar')
    # line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
    #                        cmap=plt.cm.Greys_r, lw=0)
    
    # line2 = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
    #                        cmap=plt.cm.Greys_r, lw=0)

    # ax.set_rmax(DMAX)
    # ax.grid(True)
    
    # ani = animation.FuncAnimation(fig, show, fargs=(line, line2, ), interval=50)
    # plt.show()

    data_thread.join()

    while 1:
        pass

    lidar.stop()
    lidar.disconnect()



