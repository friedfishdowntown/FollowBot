# ================= 配置文件 =================
# 所有参数集中管理

# ========== 硬件配置 ==========
# 蓝牙目标
TARGET_BLE_NAME = "FollowMe_Target"

# 串口设置
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# 摄像头设置
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
CAMERA_FOV = 75  # 视野角度 (度)

# ========== 追踪参数 ==========
# 距离控制 (cm)
SAFE_DISTANCE = 60      # 小于此距离停止
FOLLOW_DISTANCE = 100   # 理想跟随距离
MAX_DISTANCE = 200      # 超过此距离加速追赶

# 速度限制 (PWM: 0-100)
MAX_SPEED = 80          # 最大前进速度
MIN_SPEED = 30          # 最小前进速度
TURN_SPEED = 40         # 转向时的基础速度

# 画面控制
CENTER_DEADZONE = 50    # 中心死区 (像素), 在此范围内不转向
BBOX_AREA_CLOSE = 30000 # 人框面积 > 此值认为太近
BBOX_AREA_FAR = 5000    # 人框面积 < 此值认为太远

# ========== 蓝牙参数 ==========
RSSI_THRESHOLD_FOUND = -70  # 信号强度 > 此值认为找到目标
RSSI_SCAN_TIME = 5.0        # 蓝牙扫描时长 (秒)

# ========== 视觉识别 ==========
# YOLO 模型
YOLO_MODEL = "yolov8n.pt"   # nano版本, 速度快
YOLO_CONFIDENCE = 0.5       # 检测置信度阈值

# 服装颜色匹配
COLOR_MATCH_THRESHOLD = 0.7  # 颜色相似度阈值 (0-1)
NUM_INIT_SAMPLES = 3         # 初始化时采集的样本数

# 上下身分割比例 (从人框顶部开始)
UPPER_BODY_RATIO = 0.6      # 上半身占 60%
LOWER_BODY_RATIO = 0.4      # 下半身占 40%

# ========== 控制参数 ==========
# PID 转向控制
PID_KP = 0.15    # 比例系数
PID_KI = 0.01    # 积分系数
PID_KD = 0.05    # 微分系数

# 控制频率
CONTROL_LOOP_HZ = 10  # 主循环频率
CONTROL_DT = 1.0 / CONTROL_LOOP_HZ

# ========== 状态机 ==========
STATE_INIT = "INIT"           # 初始化
STATE_BLE_SEARCH = "BLE_SEARCH"  # 蓝牙搜索
STATE_ALIGN = "ALIGN"         # 对准目标
STATE_CAPTURE = "CAPTURE"     # 拍摄记录
STATE_TRACK = "TRACK"         # 视觉追踪
STATE_LOST = "LOST"           # 目标丢失
STATE_STOP = "STOP"           # 停止

# 丢失处理
LOST_FRAME_THRESHOLD = 15    # 连续丢失多少帧认为目标丢失
SEARCH_ROTATION_SPEED = 25   # 丢失后搜索旋转速度

# ========== 调试选项 ==========
DEBUG_MODE = True            # 是否打印调试信息
SAVE_DEBUG_IMAGES = False    # 是否保存调试图像
DEBUG_IMAGE_PATH = "./debug_images/"