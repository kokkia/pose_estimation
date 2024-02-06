# pose_estimation  

姿勢角推定オブザーバ
- quaternionベース
- オイラー角ベース（ZYXオイラー角 = roll, pitch, yaw）

## Installation

### Arduino
- https://github.com/bblanchon/ArduinoJson
- https://github.com/asukiaaa/MPU9250_asukiaaa

### Ubuntu Python
- Python3.8
- See [requirements.txt](requirements.txt)
```
$ pip install -r requirements.txt
```

## How to use

1. Setup arudino or esp32 environment
2. Launch esp32_imu with esp32
3. Launch observer
   ```
   $ python main.py
   ```

## Note

- 角速度センサ、加速度センサともに右手系
- 加速度センサは、平面に置いたときに、+1Gが検出される向きがZ方向の正

## Demo

https://github.com/kokkia/pose_estimation/assets/40203299/0c72cce5-2f74-4215-9034-3d6f8dc7452c

