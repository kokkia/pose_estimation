## coding: UTF-8
# 回転の座標変換
import numpy as np
import math

from transform import *
from quaternion import *

class pose_observer:
    def __init__(self, dt):
        self.q = np.array([1, 0, 0, 0])
        # time
        self.dt = dt
        # gain
        self.set_gain(20.0, 0.1)
        self.q_vec_err_sum = 0.0
        
        return
    
    def set_quaternion(self, q):
        self.q = q
    
    def set_gain(self, kp, ki):
        self.kp = kp
        self.ki = ki

    def update(self, w, g_vec_from_sensor):
        g_vec_from_q = q2g_vec(self.q)
        q_vec_err = - np.cross(g_vec_from_sensor, g_vec_from_q)
        self.q_vec_err_sum += q_vec_err * self.dt
        w = w + self.kp * q_vec_err + self.ki * self.q_vec_err_sum
        self.q = integrate_quaternion(self.q, w, self.dt)
        return self.q
    
    def get_rpy(self):
        return quaternion2rpy(self.q)