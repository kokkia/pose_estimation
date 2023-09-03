import serial
import json
import time
import numpy as np
import matplotlib.pyplot as plt
import threading
import concurrent.futures
from transform import *
from quaternion import *
from pose_observer import *
from arrow import *

class MPU9250_driver(object):
    def __init__(self):
        # serial
        self.PORT = "/dev/ttyUSB0"
        self.BAUDRATES = 115200
        self.ser = serial.Serial(self.PORT, self.BAUDRATES)

        # control
        self.dt = 0.01
        self.t = 0

        # gyro
        self.gX_offset = 0.0
        self.gY_offset = 0.0
        self.gZ_offset = 0.0
        self.w = np.array([0, 0, 0])
        self.g_vec = np.array([0, 0, 0])

        # calibration
        self.cnt = 0
        self.calibration = True
        self.gXs = []
        self.gYs = []
        self.gZs = []

        # observer
        self.q = np.array([1, 0, 0, 0])
        self.observer = pose_observer(self.dt)
        self.observer.set_quaternion(q)
    
    def set_calibration_mode(self, calibration):
        self.calibration = calibration
        return

    def get_data(self):
        byte_data = self.ser.readline()
        str_data = self.byte_data.decode().split()[0]
        try:
            json_data = json.loads(str_data)
            if len(json_data) < 6:
                return False
            aX = json_data["aX"]
            aY = json_data["aY"]
            aZ = json_data["aZ"]
            gX = json_data["gX"] + self.gX_offset
            gY = json_data["gY"] + self.gY_offset
            gZ = json_data["gZ"] + self.gZ_offset

            self.w = np.array([gX, gY, gZ])
            self.g_vec = np.array([aX, aY, aZ])
            return True
        except json.JSONDecodeError:
            print("json encode error")
            return False
    
    def calibrate_gyro_offset(self):
        self.gX_offset = 0.0
        self.gY_offset = 0.0
        self.gZ_offset = 0.0

        for i in range(100):
            if self.get_data():
                self.gXs.append(self.w[0])
                self.gYs.append(self.w[1])
                self.gZs.append(self.w[2])
            else:
                continue

        self.gX_offset = - np.mean(gXs)
        self.gY_offset = - np.mean(gYs)
        self.gZ_offset = - np.mean(gZs)
        print("calibration is done")
        return

    def estimation(self):
        while True:
            if self.get_data():
                time.sleep(self.dt)
                self.t += self.dt
                self.cnt = 0
                self.q = self.observer.update(self.w, self.g_vec)
                r, p, y = self.observer.get_rpy()
                if cnt % 5 == 0:
                    print("rpy = ", np.degrees(r), np.degrees(p), np.degrees(y), t)
            else:
                continue

    def estimate(self):
        if self.get_data():
            self.q = self.observer.update(self.w, self.g_vec)
            r, p, y = self.observer.get_rpy()
            if cnt % 5 == 0:
                print("rpy = ", np.degrees(r), np.degrees(p), np.degrees(y), t)
        else:
            pass

def estimation_thread(lock, observer):
    while True:
        time.sleep(self.dt)
        self.t += self.dt
        try:
            with lock:
                observer.estimate()
        except Exception as e:
            print(e)

def visualization_thread(lock, observer, ax, elems):
    p = np.zeros((3, 1))
    global i
    i = 0
    # for j in range(100):
    while True:
        try:
            # R = quaternion2R(observer.q)
            i += 1
            theta = np.radians(i * 10)
            R = Rz(theta)
            plt.cla()
            visualize_posture(p, R, ax, elems)
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            ax.set_xlim(-1,1)
            ax.set_ylim(-1,1)
            ax.set_zlim(-1,1)
            plt.pause(0.001)
        except Exception as e:
            print(e)

if __name__ == "__main__":

    # fig
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    elems = []

    # observer
    # observer = MPU9250_driver()
    observer = None

    # # thread
    executor = concurrent.futures.ThreadPoolExecutor(1) # 複数のスレッドを立ち上げる
    lock = threading.Lock()  # threading.Lockオブジェクトのインスタンスを1つ生成する

    # # 複数スレッドで同時に同じ処理を行う
    # executor.submit(visualization_thread, lock, observer, ax, elems)
    # # executor.submit(func2, lock)

    # visualize
    visualization_thread(lock, observer, ax, elems) # なぜかスレッドで実行すると遅い
    plt.show()