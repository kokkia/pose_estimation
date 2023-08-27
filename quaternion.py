## coding: UTF-8
import numpy as np

# q = [qw,qx,qy,qz]

def q2matrix(q):
    qmat = np.array([[  -q[1], -q[2], -q[3]],
                     [   q[0], -q[3],  q[2]],
                     [   q[3],  q[0], -q[1]],
                     [  -q[2],  q[1],  q[0]]])
    return qmat

def q2g_vec(q):
    gx = 2 * (q[1]*q[3]-q[0]*q[2])
    gy = 2 * (q[0]*q[1]+q[2]*q[3])
    gz = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
    g_vec = np.array([gx, gy, gz])
    return g_vec

def integrate_quaternion(q, w, dt):
    qmat = q2matrix(q)
    dq = 1.0 / 2.0 * np.dot(qmat, w)
    q = q + dq * dt
    norm = np.linalg.norm(q)
    q = q / norm
    return q