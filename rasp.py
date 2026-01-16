import asyncio
import cv2
import face_recognition
import threading
import time
import numpy as np
from bleak import BleakScanner
from gpiozero import Motor, DistanceSensor, OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# ================= 1. 硬件引脚配置 =================
# 指定 Pin Factory，防止 Pi 5 兼容性问题
factory = LGPIOFactory()

# --- 超声波配置 ---
# echo=GPIO24, trigger=GPIO23
sensor = DistanceSensor(echo=24, trigger=23, max_distance=4, pin_factory=factory)

# --- 电机配置 (TB6612) ---
class TB6612_Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, pin_factory=factory)
        self.in1 = OutputDevice(in1_pin, pin_factory=factory)
        self.in2 = OutputDevice(in2_pin, pin_factory=factory)
    
    def drive(self, speed):
        """speed: -1.0 到 1.0"""
        speed = max(min(speed, 1.0), -1.0)
        if speed > 0:
            self.in1.on()
            self.in2.off()
        elif speed < 0:
            self.in1.off()
            self.in2.on()
        else:
            self.in1.off()
            self.in2.off()
        self.pwm.value = abs(speed)

# 初始化左右电机
motor_left = TB6612_Motor(12, 5, 6)
motor_right = TB6612_Motor(13, 22, 27)

# ================= 2. 参数定义 =================
TAG_NAME = "FollowMe_Tag" # 你的蓝牙Tag名称
SCAN_DURATION = 12.0      # 扫描旋转时间
SCAN_SPEED = 0.4          # 扫描时的旋转速度
FOLLOW_SPEED = 0.5        # 跟随时的基础速度
RSSI_THRESHOLD = -80      # 触发扫描的信号强度
SAFE_DISTANCE_CM = 30.0   # 避障停止距离

# PID 参数 (由于颜色追踪帧率高，Kp需要适当调小防止震荡)
Kp = 0.0015               

# 全局状态
system_state = {
    "rssi": -100,
    "mode": "WAITING",
    "running": True
}

# ================= 3. 核心功能模块 =================

# --- 蓝牙扫描线程 ---
async def ble_scanner():
    print("BLE Scanner Started...")
    def callback(device, advertisement_data):
        if device.name == TAG_NAME:
            system_state["rssi"] = device.rssi
    
    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    while system_state["running"]:
        await asyncio.sleep(1)
    await scanner.stop()

def start_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_scanner())

# --- 硬件控制封装 ---
def robot_move(left_speed, right_speed):
    motor_left.drive(left_speed)
    motor_right.drive(right_speed)

def get_distance():
    d = sensor.distance
    if d is None: return 999.0
    return d * 100.0

# --- [新] 双色追踪系统 ---
class ColorTracker:
    def __init__(self):
        self.upper_body_range = None # 上衣 (lower, upper)
        self.lower_body_range = None # 裤子 (lower, upper)
        
    def get_hsv_range(self, roi):
        """计算区域的HSV范围"""
        if roi.size == 0: return None
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean_hsv = np.mean(hsv_roi, axis=(0, 1))
        H, S, V = mean_hsv
        
        # 阈值宽容度 (可根据光线调整)
        h_tol = 15   # 色相敏感度
        s_tol = 80   # 饱和度范围
        v_tol = 100  # 亮度范围
        
        lower = np.array([max(0, H - h_tol), max(20, S - s_tol), max(20, V - v_tol)])
        upper = np.array([min(180, H + h_tol), min(255, S + s_tol), min(255, V + v_tol)])
        return (lower, upper)

    def lock_colors(self, frame, face_box):
        """根据人脸位置采样上衣和裤子"""
        top, right, bottom, left = face_box
        h, w, _ = frame.shape
        face_h = bottom - top
        face_w = right - left
        
        print(f"Sampling Colors... Face: {face_box}")

        # 1. 上衣区域 (人脸正下方)
        shirt_t = min(bottom + 5, h-1)
        shirt_b = min(shirt_t + int(face_h * 1.5), h-1) # 取1.5倍脸高
        shirt_l = max(left + int(face_w * 0.1), 0)
        shirt_r = min(right - int(face_w * 0.1), w-1)
        
        if shirt_b > shirt_t and shirt_r > shirt_l:
            roi_shirt = frame[shirt_t:shirt_b, shirt_l:shirt_r]
            self.upper_body_range = self.get_hsv_range(roi_shirt)
            print(">> Upper Body Locked.")
        
        # 2. 裤子区域 (上衣下方)
        pants_t = shirt_b
        pants_b = min(pants_t + int(face_h * 1.5), h-1)
        # 假设裤子和上衣差不多宽
        pants_l = shirt_l
        pants_r = shirt_r
        
        if pants_b > pants_t and pants_r > pants_l and pants_b < h:
            roi_pants = frame[pants_t:pants_b, pants_l:pants_r]
            self.lower_body_range = self.get_hsv_range(roi_pants)
            print(">> Lower Body Locked.")
        else:
            self.lower_body_range = None
            print(">> Lower Body not visible/too close.")
            
        return True

    def find_target(self, frame):
        """追踪逻辑：返回 (cx, cy, width, debug_mask)"""
        if self.upper_body_range is None: return None
        
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # 寻找上衣
        l1, u1 = self.upper_body_range
        mask_shirt = cv2.inRange(hsv, l1, u1)
        mask_shirt = cv2.erode(mask_shirt, None, iterations=2)
        mask_shirt = cv2.dilate(mask_shirt, None, iterations=2)
        
        # 寻找裤子 (如果有)
        mask_pants = None
        if self.lower_body_range:
            l2, u2 = self.lower_body_range
            mask_pants = cv2.inRange(hsv, l2, u2)
            mask_pants = cv2.erode(mask_pants, None, iterations=2)
            mask_pants = cv2.dilate(mask_pants, None, iterations=2)
            
        # 提取轮廓
        shirt_cnts, _ = cv2.findContours(mask_shirt.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pants_cnts = []
        if mask_pants is not None:
            pants_cnts, _ = cv2.findContours(mask_pants.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        shirt_box = None
        pants_box = None
        
        if shirt_cnts:
            c = max(shirt_cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: shirt_box = cv2.boundingRect(c)
            
        if pants_cnts:
            c = max(pants_cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: pants_box = cv2.boundingRect(c)

        # 决策融合
        debug_mask = mask_shirt
        if shirt_box and pants_box:
            sx, sy, sw, sh = shirt_box
            px, py, pw, ph = pants_box
            # 如果裤子在下面，且水平接近，取平均
            if py > sy and abs((sx+sw/2) - (px+pw/2)) < 150:
                final_x = (sx + sw//2 + px + pw//2) // 2
                final_y = (sy + sh//2 + py + ph//2) // 2
                final_w = max(sw, pw)
                if mask_pants is not None: debug_mask = cv2.bitwise_or(mask_shirt, mask_pants)
                return (final_x, final_y, final_w, debug_mask)

        # 降级：只有上衣
        if shirt_box:
            x, y, w, h = shirt_box
            return (x + w//2, y + h//2, w, mask_shirt)
            
        # 降级：只有裤子 (背对模式)
        if pants_box:
            x, y, w, h = pants_box
            return (x + w//2, y + h//2, w, mask_pants)
            
        return None

# --- 雷达扫描系统 ---
class RadarSystem:
    def __init__(self):
        self.rssi_log = []
        self.visual_log = []
        self.start_time = 0

    def start(self):
        self.rssi_log = []
        self.visual_log = []
        self.start_time = time.time()

    def record(self, frame, rssi):
        now = time.time()
        self.rssi_log.append({'t': now, 'rssi': rssi})
        # 保持0.5倍缩放，确保采样颜色时有足够像素
        small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
        self.visual_log.append({'t': now, 'img': small})

    def analyze(self):
        print("Analyzing Scan Data...")
        if not self.rssi_log: return None, None
        rssi_vals = [x['rssi'] for x in self.rssi_log]
        if len(rssi_vals) < 5: return None, None
        
        # 找最强信号
        smoothed = np.convolve(rssi_vals, np.ones(5)/5, mode='valid')
        peak_idx = np.argmax(smoothed) + 2
        peak_time = self.rssi_log[peak_idx]['t']
        
        # 找对应帧
        best_frame_data = min(self.visual_log, key=lambda x: abs(x['t'] - peak_time))
        frame = best_frame_data['img']
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 识别人脸
        boxes = face_recognition.face_locations(rgb)
        if not boxes: return None, None
        
        # 选最中间的脸
        h, w, _ = frame.shape
        center_x = w // 2
        best_box = min(boxes, key=lambda b: abs((b[3]+b[1])/2 - center_x))
        encoding = face_recognition.face_encodings(rgb, [best_box])[0]
        
        # 返回: 编码用于验证(可选), (图片, 坐标)用于颜色提取
        return encoding, (frame, best_box)

# ================= 4. 主程序 =================
def main():
    t_ble = threading.Thread(target=start_ble_thread, daemon=True)
    t_ble.start()

    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    cap.set(3, 640); cap.set(4, 480)
    
    radar = RadarSystem()
    color_tracker = ColorTracker()
    
    lost_counter = 0
    target_width_ma = 0 # 宽度的移动平均
    
    print("System Dual-Color-Track Ready.")

    try:
        while system_state["running"]:
            ret, frame = cap.read()
            if not ret: break
            
            mode = system_state["mode"]
            dist_cm = get_distance()
            status_text = mode

            # --- 状态机逻辑 ---
            
            # 1. 等待模式
            if mode == "WAITING":
                robot_move(0, 0)
                status_text = f"WAIT RSSI:{system_state['rssi']}"
                if system_state['rssi'] > RSSI_THRESHOLD:
                    system_state["mode"] = "SCANNING"
                    radar.start()

            # 2. 扫描模式
            elif mode == "SCANNING":
                elapsed = time.time() - radar.start_time
                status_text = f"SCAN: {elapsed:.1f}s"
                robot_move(-SCAN_SPEED, SCAN_SPEED) # 原地旋转
                radar.record(frame, system_state['rssi'])

                if elapsed > SCAN_DURATION:
                    robot_move(0, 0)
                    enc, data = radar.analyze()
                    
                    if enc is not None:
                        scan_frame, face_box = data
                        # 尝试锁定颜色
                        if color_tracker.lock_colors(scan_frame, face_box):
                            system_state["mode"] = "LOCKED"
                            lost_counter = 0
                        else:
                            print("Face found but color sampling failed.")
                            system_state["mode"] = "WAITING"
                    else:
                        print("No target found.")
                        system_state["mode"] = "WAITING"

            # 3. 锁定模式 (颜色跟随)
            elif mode == "LOCKED":
                # 安全避障
                if dist_cm < SAFE_DISTANCE_CM:
                    robot_move(0, 0)
                    status_text = "OBSTACLE!"
                    cv2.putText(frame, "STOP", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
                
                else:
                    # 获取颜色追踪结果
                    res = color_tracker.find_target(frame)
                    
                    if res:
                        # 找到目标
                        lost_counter = 0
                        cx, cy, w, mask = res
                        
                        # 显示 Mask (右上角小图)
                        mask_small = cv2.resize(mask, (160, 120))
                        mask_c = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
                        frame[0:120, 480:640] = mask_c
                        
                        # 绘制目标框
                        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                        
                        # PID 转向计算
                        error = cx - 320
                        turn = error * Kp
                        
                        # 距离保持 (基于色块宽度)
                        # 宽度越大说明越近
                        TARGET_WIDTH = 180 
                        throttle = 0
                        
                        if w < TARGET_WIDTH - 30: 
                            throttle = FOLLOW_SPEED     # 前进
                        elif w > TARGET_WIDTH + 30: 
                            throttle = -FOLLOW_SPEED/2  # 后退
                        
                        # 混合输出
                        l_out = throttle + turn
                        r_out = throttle - turn
                        
                        # 归一化保护
                        l_out = max(min(l_out, 1.0), -1.0)
                        r_out = max(min(r_out, 1.0), -1.0)
                        
                        robot_move(l_out, r_out)
                        status_text = f"Track W:{w}"
                    
                    else:
                        # 丢失目标
                        robot_move(0, 0)
                        lost_counter += 1
                        status_text = f"LOST {lost_counter}"
                        
                        if lost_counter > 60: # 约2秒没看到颜色
                            print("Target Lost completely.")
                            system_state["mode"] = "WAITING"

            # UI 绘制
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Dist:{dist_cm:.0f}cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            cv2.imshow("FollowBot View", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                system_state["running"] = False
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping...")
        robot_move(0, 0)
        sensor.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()