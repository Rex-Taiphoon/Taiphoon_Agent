# PX4 建構參考指南

## 常用建構指令

### 建構 SITL 版本（軟體在環模擬）

```bash
cd ~/PX4-Autopilot
make px4_sitl
```

### 建構特定機型

```bash
# 四軸飛行器（預設）
make px4_sitl gazebo-classic_iris

# 標準 VTOL
make px4_sitl gazebo-classic_standard_vtol

# 固定翼
make px4_sitl gazebo-classic_plane
```

### 清除建構快取

```bash
cd ~/PX4-Autopilot
make clean
# 或清除特定 target 的建構輸出
make px4_sitl clean
```

### 平行編譯（加快建構速度）

```bash
cd ~/PX4-Autopilot
make px4_sitl -j$(nproc)
```

## 建構輸出位置

- SITL 執行檔：`~/PX4-Autopilot/build/px4_sitl_default/bin/px4`
- 設定檔與暫存檔：`~/PX4-Autopilot/build/px4_sitl_default/rootfs/`

## 常見建構問題

### 問題 1：編譯過程中記憶體不足

**解決方法**：
- 關閉其他占用記憶體的應用程式
- 使用 `make px4_sitl -j2` 減少平行編譯數量
- 在 WSL2 的 `.wslconfig` 中增加記憶體分配

### 問題 2：子模組更新失敗

```bash
cd ~/PX4-Autopilot
git submodule update --recursive --init --force
```

### 問題 3：Python 相依套件衝突

建議使用虛擬環境：
```bash
python3 -m venv ~/px4_venv
source ~/px4_venv/bin/activate
pip install jinja2 toml numpy empy packaging kconfiglib
```

### 問題 4：git 複製中斷

```bash
cd ~
rm -rf PX4-Autopilot  # 清除不完整的複製
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

## 環境變數

| 環境變數 | 說明 | 預設值 |
|----------|------|--------|
| `PX4_SIM_MODEL` | 模擬模型名稱 | `sihsim_quadx` |
| `PX4_SIMULATOR` | 模擬器類型 | `sihsim` |
| `PX4_HOME_LAT` | 起始緯度 | `47.397742` |
| `PX4_HOME_LON` | 起始經度 | `8.545594` |
| `PX4_UXRCE_DDS_PORT` | uXRCE-DDS 埠號 | `8888` |
| `PX4_UXRCE_DDS_NS` | ROS2 命名空間 | `fmu` |
