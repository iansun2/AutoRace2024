from rplidar import RPLidar
import threading

mutex = threading.Lock()

lidar_target = 0

def lidar_handler():
    global lidar_target

    PORT = "/dev/ttyUSB1"
    # init Lidar
    lidar = RPLidar(PORT)
    iterator = lidar.iter_scans()

    fov = 90
    l_edge = fov / 2
    r_edge = 360 - fov / 2
    filt_dist = 300
    kp = 5e-3

    while 1:
        err = 0
        try:
            scan = next(iterator)
        except:
            lidar.stop()
            lidar.disconnect()
            lidar = RPLidar(PORT)
            iterator = lidar.iter_scans()
            continue

        for data in scan:
            deg = data[1]
            dist = data[2]
            if dist < filt_dist:
                if deg >= r_edge:
                    err += (deg - r_edge) * (dist - filt_dist)
                elif deg < l_edge:
                    err += (deg - l_edge) * (dist - filt_dist)
        #print("error: ", err * kp)
        mutex.acquire()
        lidar_target = err * kp
        mutex.release()


def get_lidar_target():
    mutex.acquire()
    val = lidar_target
    mutex.release()
    return int(val)



lidar_thread = threading.Thread(target=lidar_handler)
lidar_thread.start()



if __name__ == '__main__':
    #lidar = RPLidar('/dev/ttyUSB1')
    #iterator = lidar.iter_scans()

    while 1:
        err = get_lidar_target()
        print("error: ", err)





