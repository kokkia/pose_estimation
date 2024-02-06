## coding: UTF-8
# 姿勢角推定
import numpy as np
import math

from transform import *
from quaternion import *
from utility import *

class pose_observer_quaternion:
    def __init__(self, dt):
        self.q = np.array([1, 0, 0, 0])
        # time
        self.dt = dt
        # gain
        self.set_gain(5.0, 1.0)
        self.g_vec_err_sum = 0.0
        return
    
    def set_quaternion(self, q):
        self.q = q
    
    def set_gain(self, kp, ki):
        self.kp = kp
        self.ki = ki

    def update(self, w, g_vec_from_sensor):
        g_vec_from_q = q2g_vec(self.q)
        norm = np.linalg.norm(g_vec_from_sensor)
        g_vec_from_sensor = g_vec_from_sensor / norm
        g_vec_err = np.cross(g_vec_from_sensor, g_vec_from_q)
        g_vec_err = np.arcsin(g_vec_err) # 線形のFBに補正
        self.g_vec_err_sum += g_vec_err * self.dt
        w = w + self.kp * g_vec_err + self.ki * self.g_vec_err_sum
        self.q = integrate_quaternion(self.q, w, self.dt)
        return self.q
    
    def get_rpy(self):
        return quaternion2rpy(self.q)
    
    def get_R(self):
        return quaternion2R(self.q)

class pose_observer_euler:
    def __init__(self, dt):
        self.rpy = np.array([0.0, 0.0, 0.0])
        # time
        self.dt = dt
        # gain
        self.set_gain(5.0, 1.0)
        self.g_vec_err_sum = 0.0
        # limit
        self.roll_max = np.radians(80)
        self.pitch_max = np.radians(80)
        return
    
    def set_quaternion(self, q):
        r, p, y = quaternion2rpy(q)
        self.rpy = np.array([r, p, y])
    
    def set_rpy(r, p, y):
        self.rpy = np.array([r, p, y])

    def set_gain(self, kp, ki):
        self.kp = kp
        self.ki = ki

    def rpy2g_vec(self, rpy):
        roll = rpy[0]
        pitch = rpy[1]
        gx = -np.sin(pitch)
        gy = np.sin(roll) * np.cos(pitch)
        gz = np.cos(roll) * np.cos(pitch)
        g_vec = np.array([gx, gy, gz])
        return g_vec
    
    def integrate_rpy(self, rpy, w, dt):
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        # zero回避
        cosPitch = np.cos(pitch)
        if abs(cosPitch) < 1e-4:
            cosPitch = 1e-4
        jacobian = np.array([[1, np.sin(roll) * np.tan(pitch), np.cos(roll) * np.tan(pitch)],
                             [0, np.cos(roll),              -np.sin(roll)],
                             [0, np.sin(roll)/cosPitch, np.cos(roll)/cosPitch]])
        drpy = np.dot(jacobian, w)
        rpy = rpy + drpy * dt
        return rpy

    def update(self, w, g_vec_from_sensor):
        # 重力ベクトル計算
        g_vec_from_rpy = self.rpy2g_vec(self.rpy)
        norm = np.linalg.norm(g_vec_from_sensor)
        # 加速度センサの重力ベクトル
        g_vec_from_sensor = g_vec_from_sensor / norm
        # 偏差計算
        g_vec_err = np.cross(g_vec_from_sensor, g_vec_from_rpy)
        g_vec_err = np.arcsin(g_vec_err) # 線形のFBに補正
        self.g_vec_err_sum += g_vec_err * self.dt
        # 補正
        w = w + self.kp * g_vec_err + self.ki * self.g_vec_err_sum
        self.rpy = self.integrate_rpy(self.rpy, w, self.dt)
        # self.rpy = self.limit_rpy(self.rpy)
        return self.rpy
    
    def limit_rpy(self, rpy):
        # 補正がおかしくなるため、これ使わないほうがいい
        roll = limit_range(-self.roll_max, self.roll_max, rpy[0])
        pitch = limit_range(-self.pitch_max, self.pitch_max, rpy[1])
        yaw = pi2pi(rpy[2])
        rpy = np.array([roll, pitch, yaw])
        return rpy
    
    def get_rpy(self):
        return self.rpy[0], self.rpy[1], self.rpy[2]
    
    def get_R(self):
        return rpy2R(self.rpy[0], self.rpy[1], self.rpy[2])

    def get_quaternion(self):
        return rpy2quaternion(self.rpy[0], self.rpy[1], self.rpy[2])