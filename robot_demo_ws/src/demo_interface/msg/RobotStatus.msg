# 机器人状态消息定义
# 包含机器人的位置、速度、电池等状态信息

# 时间戳
builtin_interfaces/Time timestamp

# 机器人ID
string robot_id

# 位置信息 (x, y, z坐标，单位：米)
geometry_msgs/Point position

# 姿态信息 (四元数表示)
geometry_msgs/Quaternion orientation

# 线速度 (m/s)
geometry_msgs/Vector3 linear_velocity

# 角速度 (rad/s)
geometry_msgs/Vector3 angular_velocity

# 电池电量 (百分比 0-100)
float32 battery_level

# 工作模式
string mode

# 机器人状态
uint8 status

# 状态常量定义
uint8 STATUS_IDLE = 0
uint8 STATUS_MOVING = 1
uint8 STATUS_CHARGING = 2
uint8 STATUS_ERROR = 3

# 是否在充电
bool is_charging

# 错误代码 (0表示无错误)
uint32 error_code

# 状态描述
string status_message