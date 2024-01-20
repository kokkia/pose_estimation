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

## How to use

1. Setup arudino or esp32 environment
2. Launch esp32_imu with esp32
3. Launch observer
   ```
   $ python main.py
   ```

## Demo

https://github.com/kokkia/pose_estimation/assets/40203299/0c72cce5-2f74-4215-9034-3d6f8dc7452c

