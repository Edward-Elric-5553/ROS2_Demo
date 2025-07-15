# 运动控制命令消息定义
# 用于控制机器人的运动

# 时间戳
builtin_interfaces/Time timestamp

# 命令ID
string command_id

# 线速度 (m/s)
geometry_msgs/Vector3 linear_velocity

# 角速度 (rad/s)
geometry_msgs/Vector3 angular_velocity

# 目标位置 (可选，用于位置控制)
geometry_msgs/Point target_position

# 目标姿态 (可选，用于姿态控制)
geometry_msgs/Quaternion target_orientation

# 控制模式
uint8 control_mode

# 控制模式常量定义
uint8 MODE_VELOCITY = 0    # 速度控制模式
uint8 MODE_POSITION = 1    # 位置控制模式
uint8 MODE_STOP = 2        # 停止模式

# 优先级 (0-10, 10为最高优先级)
uint8 priority

# 执行时间 (秒，0表示持续执行直到新命令)
float32 duration

# 是否为紧急停止命令
bool emergency_stop

# 命令描述
string description