# PX4 SITL 連接埠對應表

## 基本規則

- uXRCE-DDS 埠號：`8888 + 實例編號`
- MAVLink GCS 埠號：`18570 + 實例編號`
- MAV_SYS_ID：`實例編號 + 1`
- ROS2 命名空間：`fmu{實例編號 + 1}`

## 對應表

| 實例編號 | uXRCE-DDS 埠號 | MAVLink GCS 埠號 | MAV_SYS_ID | ROS2 命名空間 |
|----------|---------------|------------------|------------|----------------|
| 0        | 8888          | 18570            | 1          | fmu1           |
| 1        | 8889          | 18571            | 2          | fmu2           |
| 2        | 8890          | 18572            | 3          | fmu3           |
| 3        | 8891          | 18573            | 4          | fmu4           |
| 4        | 8892          | 18574            | 5          | fmu5           |
| 5        | 8893          | 18575            | 6          | fmu6           |
| N        | 8888+N        | 18570+N          | N+1        | fmu{N+1}       |

## 環境變數說明

| 環境變數 | 說明 | 範例值 |
|----------|------|--------|
| `PX4_SIM_MODEL` | 模擬模型 | `sihsim_quadx` |
| `PX4_SIMULATOR` | 模擬器類型 | `sihsim` |
| `PX4_UXRCE_DDS_PORT` | uXRCE-DDS 連接埠 | `8888` |
| `PX4_UXRCE_DDS_NS` | ROS2 命名空間 | `fmu1` |
| `PX4_NET_INTERFACE` | 網路介面 | `eth0` |
| `PX4_PARAM_MAV_0_BROADCAST` | MAVLink 廣播模式 | `1` |
| `PX4_HOME_LAT` | 起始緯度 | `25.03996592258936` |
| `PX4_HOME_LON` | 起始經度 | `121.51195163336611` |

## 位置計算

實例間距 3 公尺，從 `start_single_instance.sh` 內建的 Python 計算：

```python
import math
BASE_LAT = 25.03996592258936  # 台北
BASE_LON = 121.51195163336611
lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(BASE_LAT)))
offset = (N - 1) * 3.0 * lon_per_m  # N = 實例編號
```
