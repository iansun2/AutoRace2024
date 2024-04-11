from rplidar import RPLidar
import threading

mutex = threading.Lock()

lidar_target = 0
lidar_closest = [0, 0] # dist, angle
lidar_deg_view = [0] * 360

def lidar_handler():
    global lidar_target, lidar_closest, lidar_deg_view

    PORT = "/dev/ttyUSB0"
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
            print('lidar err')
            lidar.stop()
            lidar.disconnect()
            lidar = RPLidar(PORT)
            iterator = lidar.iter_scans()
            continue

        closest = [1000000, 0]

        deg_view = [0] * 360
        deg_data_cnt = 0
        last_deg = -1000
        insert_list = []
        update_mask = [0] * 360
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

            # deg as index
            deg = int(deg)
            update_mask[deg] = 1

            # new scan, update deg
            if last_deg == -1000:
                last_deg = deg
            # new data, calculate last deg
            elif last_deg != deg:
                deg_view[last_deg] /= deg_data_cnt
                deg_data_cnt = 0
                # need insert
                if deg - last_deg != 1:
                    insert_list.append([last_deg, deg])
                    update_mask[last_deg:deg] = [1] * (deg - last_deg)
                last_deg = deg
                
            # count data in this deg
            deg_data_cnt += 1
            
            # first data in this deg
            if deg_data_cnt == 1:
                deg_view[deg] = dist
            # other sample in this deg
            else:
                deg_view[deg] += dist

            # last data, calculate current deg
            if data == scan[-1]:
                deg_view[deg] /= deg_data_cnt

        # insert process
        for insert in insert_list:
            delta_deg = insert[1] - insert[0]
            dist_step = (deg_view[insert[1]] - deg_view[insert[0]]) / delta_deg
            for deg in range(insert[0]+1, insert[1]):
                deg_view[deg] = deg_view[deg-1] + dist_step


        #print("error: ", err * kp)
        mutex.acquire()
        lidar_target = err * kp
        lidar_closest = closest.copy()

        for deg in range(0, 360):
            if update_mask[deg]:
                lidar_deg_view[deg] = deg_view[deg]
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
    #print(lidar_view)
    val = lidar_deg_view.copy()
    #print(val)
    mutex.release()
    return val



lidar_thread = threading.Thread(target=lidar_handler)
lidar_thread.start()



if __name__ == '__main__':
    #lidar = RPLidar('/dev/ttyUSB1')
    #iterator = lidar.iter_scans()

    while 1:
        err = get_lidar_target()
        #print("error: ", err)
        closest = get_lidar_closest()
        #print("closest: ", closest)
        pass






