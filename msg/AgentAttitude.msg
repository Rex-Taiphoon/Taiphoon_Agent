# Taiphoon Agent 姿態訊息
# 發布於 /{agent_namespace}/attitude，頻率由 state_publish_rate_hz 參數控制（預設 10Hz）

builtin_interfaces/Time stamp   # 訊息時間戳
float32[4] q                    # 四元數姿態（w, x, y, z）
