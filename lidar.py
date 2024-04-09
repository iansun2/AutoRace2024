from rplidar import RPLidar
import threading

mutex = threading.Lock()

lidar_target = 0
lidar_closest = [0, 0] # dist, angle
lidar_view = [0, 0]

def lidar_handler():
    global lidar_target, lidar_closest, lidar_view

    PORT = "/dev/ttyUSB1"
    # init Lidar
    lidar = RPLidar(PORT)
    iterator = lidar.iter_scans()

    fov = 160
    l_edge = fov / 2
    r_edge = 360 - fov / 2
    filt_dist = 500
    kp = 5e-4

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

        closest = [1000000, 0]
        view = []

        for data in scan:
            deg = data[1]
            dist = data[2]
            if dist < filt_dist:
                # get closest
                if dist < closest[0]:
                    closest = [dist, deg]
                # avoidance
                if deg >= r_edge:
                    err += (deg - r_edge) * (dist - filt_dist)
                elif deg < l_edge:
                    err += (deg - l_edge) * (dist - filt_dist)
            
            view.append([deg, dist])

        #print("error: ", err * kp)
        mutex.acquire()
        lidar_target = err * kp
        lidar_closest = closest.copy()
        lidar_view = view.copy()
        mutex.release()


def get_lidar_target():
    mutex.acquire()
    val = lidar_target
    mutex.release()
    return int(val)


def get_lidar_closest():
    mutex.acquire()
    val = lidar_closest
    mutex.release()
    return val


def get_lidar_view():
    mutex.acquire()
    val = lidar_view.copy()
    mutex.release()
    return val



lidar_thread = threading.Thread(target=lidar_handler)
lidar_thread.start()



if __name__ == '__main__':
    #lidar = RPLidar('/dev/ttyUSB1')
    #iterator = lidar.iter_scans()

    while 1:
        err = get_lidar_target()
        print("error: ", err)
        closest = get_lidar_closest()
        print("closest: ", closest)






