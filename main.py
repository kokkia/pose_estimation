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
        self.mode = 'quaternion'
        # self.mode = 'rpy'
        self.q = np.array([1, 0, 0, 0])
        r, p, y = quaternion2rpy(self.q)
        self.rpy = np.array([r, p, y])
        if self.mode == 'quaternion':
            self.observer = pose_observer_quaternion(self.dt)
        elif self.mode == 'rpy':
            self.observer = pose_observer_euler(self.dt)
        self.observer.set_quaternion(self.q)
    
    def get_data(self):
        byte_data = self.ser.readline()
        str_data = byte_data.decode().split()[0]
        try:
            json_data = json.loads(str_data)
            if len(json_data) < 6:
                return False
            # ロボットの動座標系に補正
            aX = -json_data["aX"]
            aY = -json_data["aY"]
            aZ = -json_data["aZ"]
            gX = - json_data["gX"]*1.5 + self.gX_offset
            gY = - json_data["gY"]*1.5 + self.gY_offset
            gZ = - json_data["gZ"]*1.5 + self.gZ_offset # z方向だけ逆

            self.w = np.array([gX, gY, gZ])
            self.g_vec = np.array([aX, aY, aZ])
            return True
        except json.JSONDecodeError:
            # print("json encode error")
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
            time.sleep(0.01)

        self.gX_offset = - np.mean(self.gXs)
        self.gY_offset = - np.mean(self.gYs)
        self.gZ_offset = - np.mean(self.gZs)
        print("calibration is done")
        return

    def estimation(self):
        while True:
            if self.get_data():
                time.sleep(self.dt)
                self.t += self.dt
                self.cnt += 1
                if self.mode == 'quaternion':
                    self.q = self.observer.update(self.w, self.g_vec)
                elif self.mode == 'rpy':
                    self.rpy = self.observer.update(self.w, self.g_vec)
                    self.q = self.observer.get_quaternion()
                else:
                    print("you should set mode")
                    break
                r, p, y = self.observer.get_rpy()
                if self.cnt % 5 == 0:
                    print("rpy = ", np.degrees(r), np.degrees(p), np.degrees(y), self.t)
            else:
                continue

    def estimate(self):
        if self.get_data():
            if self.mode == 'quaternion':
                self.q = self.observer.update(self.w, self.g_vec)
            elif self.mode == 'rpy':
                self.rpy = self.observer.update(self.w, self.g_vec)
                self.q = self.observer.get_quaternion()
            else:
                print("you should set mode")
            if self.cnt % 10 == 0:
                r, p, y = self.observer.get_rpy()
                print("rpy = ", np.degrees(r), np.degrees(p), np.degrees(y), self.t)
        else:
            pass

def estimation_thread(lock, observer):
    while True:
        try:
            time.sleep(observer.dt*0.4) # realtime性がないため調整
            observer.t += observer.dt
            observer.cnt += 1
            with lock:
                observer.estimate()
        except Exception as e:
            print(e)

def visualization_thread(lock, observer, ax, elems):
    p = np.zeros((3, 1))
    while True:
        try:
            R = quaternion2R(observer.q)
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
            plt.pause(0.1)
        except Exception as e:
            print(e)

if __name__ == "__main__":

    # fig
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    elems = []

    # observer
    observer = MPU9250_driver()
    observer.calibrate_gyro_offset()

    # thread
    executor = concurrent.futures.ThreadPoolExecutor(1) # 複数のスレッドを立ち上げる
    lock = threading.Lock()  # threading.Lockオブジェクトのインスタンスを1つ生成する

    # # # 複数スレッドで同時に同じ処理を行う
    executor.submit(estimation_thread, lock, observer)

    # # visualize
    visualization_thread(lock, observer, ax, elems) # なぜかスレッドで実行すると遅い
    plt.show()