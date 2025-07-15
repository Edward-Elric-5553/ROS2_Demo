# 传感器数据消息定义
# 包含各种传感器的数据信息

# 时间戳
builtin_interfaces/Time timestamp

# 传感器ID
string sensor_id

# 温度传感器数据 (摄氏度)
float32 temperature

# 湿度传感器数据 (百分比)
float32 humidity

# 距离传感器数据 (米)
float32 distance

# 光照强度 (lux)
float32 light_intensity

# 传感器状态 (0: 正常, 1: 警告, 2: 错误)
uint8 status

# 状态常量定义
uint8 STATUS_OK = 0
uint8 STATUS_WARNING = 1
uint8 STATUS_ERROR = 2

# 额外信息
string info