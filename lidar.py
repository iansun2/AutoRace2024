from rplidar import RPLidar
import multiprocessing
import numpy as np
import copy
import time



class Lidar():
    def __init__(self):
        
        self.lidar_deg = multiprocessing.Array('d', [0] * 1000)
        self.lidar_dist = multiprocessing.Array('i', [0] * 1000)
        self.lidar_data_len = multiprocessing.Value('i', 0)
        self.mutex = multiprocessing.Lock()
        
        multiprocessing.Process(target=self._measure, args= (self.mutex, self.lidar_deg, self.lidar_dist, self.lidar_data_len, )).start()
        print('end init')


    def _measure(self, mutex, lidar_deg, lidar_dist, lidar_data_len):
        #PORT = "COM9"
        PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        #PORT = '/dev/ttyUSB0'
        lidar = RPLidar(PORT)
        lidar.stop()
        iterator = lidar.iter_scans(max_buf_meas=3000)
        while(1):
            measure_success = False
            idx = 0

            try:
                #print('lidar get measure', time.time())
                scan = next(iterator)
                measure_success = True
            except Exception as e:
                print('lidar err', e)
                lidar = RPLidar(PORT)
                lidar.stop()
                lidar.disconnect()
                iterator = lidar.iter_scans(max_buf_meas=3000)
            
            if measure_success:
                mutex.acquire()
                for data in scan:
                    deg = data[1]
                    dist = data[2]
                    lidar_deg[idx] = deg
                    lidar_dist[idx] = int(dist)
                    idx += 1
                lidar_data_len.value = len(scan)
                mutex.release() 

    
    def get_angle_data(self):
        self.mutex.acquire()
        var = []
        for idx in range(0, self.lidar_data_len.value):
            var.append([self.lidar_deg[idx], self.lidar_dist[idx]])
            #print(idx, var[-1])
        self.mutex.release()
        return var

    
    def get_avoidance(self, config: list) -> int: # config: [fov, filt_dist, kp]
        fov = config[0]
        l_edge = fov / 2
        r_edge = 360 - fov / 2
        filt_dist = config[1]
        err = 0

        self.mutex.acquire()
        #print(self.lidar_data_len.value)
        for idx in range(0, self.lidar_data_len.value):
            dist = self.lidar_dist[idx]
            deg = self.lidar_deg[idx]
            if dist < filt_dist:
                if deg >= r_edge:
                    err += (deg - r_edge) * (dist - filt_dist)
                elif deg < l_edge:
                    err += (deg - l_edge) * (dist - filt_dist)
        self.mutex.release()
        return int(config[2] * err)


    def get_closest(self) -> list:
        self.mutex.acquire()
        closest = [1e10, 0]
        filt_dist = 500
        for idx in range(0, self.lidar_data_len.value):
            dist = self.lidar_dist[idx]
            deg = self.lidar_deg[idx]
            if dist < filt_dist and dist < closest[0]:
                closest = [dist, deg]
        self.mutex.release()
        return copy.deepcopy(closest)





if __name__ == '__main__':
    lidar = Lidar()
    #multiprocessing.freeze_support()
    last = time.time()
    while 1:
        #lidar.measure()
        #lidar_thread.join()
        
        
        if time.time() - last > 0.3:
            err = lidar.get_avoidance(config=[120, 500, 5.6e-4])
            #print("error: ", err)
            closest = lidar.get_closest()
            print("closest: ", closest)
            data = lidar.get_angle_data()
            #print(np.array(data))
            last = time.time()
        
        pass






