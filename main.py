import serial
import json
import time
import numpy as np

from transform import *
from quaternion import *
from pose_observer import *

# serial
PORT = "/dev/ttyUSB0"
BAUDRATES = 115200
ser = serial.Serial(PORT, BAUDRATES)

# control
dt = 0.01
t = 0

# gyro
gX_offset = 0.0
gY_offset = 0.0
gZ_offset = 0.0

# calibration
cnt = 0
calibration = True
gXs = []
gYs = []
gZs = []

# observer
q = np.array([1, 0, 0, 0])
observer = pose_observer(dt)
observer.set_quaternion(q)

while True:
    byte_data = ser.readline()
    str_data = byte_data.decode().split()[0]
    time.sleep(dt)
    t += dt
    cnt += 1
    try:
        json_data = json.loads(str_data)
        if len(json_data) < 6:
            continue
        aX = json_data["aX"]
        aY = json_data["aY"]
        aZ = json_data["aZ"]
        gX = json_data["gX"] + gX_offset
        gY = json_data["gY"] + gY_offset
        gZ = json_data["gZ"] + gZ_offset

        w = np.array([gX, gY, gZ])
        g_vec = np.array([aX, aY, aZ])

        if calibration:
            gXs.append(gX)
            gYs.append(gY)
            gZs.append(gZ)
            if cnt > 100:
                gX_offset = - np.mean(gXs)
                gY_offset = - np.mean(gYs)
                gZ_offset = - np.mean(gZs)
                print("calibration is done")
                calibration = False
        else:
            q = observer.update(w, g_vec)
            r, p, y = observer.get_rpy()
            if cnt % 5 == 0:
                print("rpy = ", np.degrees(r), np.degrees(p), np.degrees(y), t)

    except json.JSONDecodeError:
        print("encode error")
        continue
