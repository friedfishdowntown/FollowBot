import asyncio
import cv2
import face_recognition
import serial
import threading
import time
import numpy as np
from bleak import BleakScanner

# ================= 配置参数 =================
SERIAL_PORT = '/dev/ttyACM0'  # 检查你的端口 ls /dev/tty*
BAUD_RATE = 115200
TAG_NAME = "FollowMe_Tag"

# 扫描与运动
SCAN_DURATION = 12.0     # 旋转一圈耗时 (秒)
SCAN_SPEED_PWM = 300     # 旋转时的电机速度
FOLLOW_SPEED_PWM = 500   # 跟随速度
RSSI_THRESHOLD = -80     # 启动扫描的信号阈值
SAFE_DISTANCE = 30.0     # 避障距离 (cm)

# PID 参数 (视觉转向)
Kp = 0.4

# ================= 全局状态变量 =================
system_state = {
    "rssi": -100,           # 当前蓝牙信号强度
    "distance": 999.0,      # 当前超声波距离
    "mode": "WAITING",      # 模式: WAITING, SCANNING, LOCKED
    "running": True
}

# ================= 1. 蓝牙扫描 (异步线程) =================
async def ble_scanner():
    """ 持续扫描特定名称的蓝牙设备 """
    print("🔵 BLE Scanner Started...")
    def callback(device, advertisement_data):
        if device.name == TAG_NAME:
            system_state["rssi"] = device.rssi
            # 调试用: print(f"BLE Signal: {device.rssi}")

    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    while system_state["running"]:
        await asyncio.sleep(1)
    await scanner.stop()

def start_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_scanner())

# ================= 2. STM32 通信 (串口线程) =================
class STM32Interface:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
            print("✅ STM32 Connected.")
        except:
            print("⚠️ STM32 Not Found! (Running in Sim Mode)")
            self.ser = None

    def update_loop(self):
        while system_state["running"]:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "Distance" in line:
                        # 格式 "Distance: 25.4"
                        dist = float(line.split(':')[1].strip())
                        system_state["distance"] = dist
                except: pass
            time.sleep(0.02)

    def move(self, left, right):
        if self.ser:
            cmd = f"L:{int(left)},R:{int(right)}\n"
            self.ser.write(cmd.encode())

# ================= 3. 雷达认主算法 =================
class RadarSystem:
    def __init__(self):
        self.rssi_log = []   # (time, rssi)
        self.visual_log = [] # (time, image_roi)
        self.start_time = 0

    def start(self):
        self.rssi_log = []
        self.visual_log = []
        self.start_time = time.time()

    def record(self, frame, rssi):
        now = time.time()
        self.rssi_log.append({'t': now, 'rssi': rssi})
        
        # 为了不卡顿，只存缩小后的图
        small = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
        # 存整张图，后面再切脸，防止此刻没检测到
        self.visual_log.append({'t': now, 'img': small})

    def analyze(self):
        """ 找出信号最强时刻对应的人脸特征 """
        print("📊 Analyzing Scan Data...")
        if not self.rssi_log: return None

        # 平滑信号
        rssi_vals = [x['rssi'] for x in self.rssi_log]
        if len(rssi_vals) < 5: return None
        smoothed = np.convolve(rssi_vals, np.ones(5)/5, mode='valid')
        
        # 找峰值
        peak_idx = np.argmax(smoothed) + 2
        peak_time = self.rssi_log[peak_idx]['t']
        peak_rssi = self.rssi_log[peak_idx]['rssi']
        print(f"📈 Peak RSSI: {peak_rssi} at t={peak_time:.2f}")

        # 找对应时间的图片
        best_frame_data = min(self.visual_log, key=lambda x: abs(x['t'] - peak_time))
        
        # 从这张关键帧里提取正中间的人脸
        frame = best_frame_data['img']
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 检测人脸
        boxes = face_recognition.face_locations(rgb)
        if not boxes: 
            print("❌ Signal Peak found, but no face visible.")
            return None
        
        # 找最中间的脸 (画面宽 160, 中心 80)
        h, w, _ = frame.shape
        center_x = w // 2
        best_box = min(boxes, key=lambda b: abs((b[3]+b[1])/2 - center_x))
        
        # 提取特征
        encoding = face_recognition.face_encodings(rgb, [best_box])[0]
        print("✅ Master Logic Locked!")
        return encoding

# ================= 4. 主逻辑 (Main Brain) =================
def main():
    # --- 启动线程 ---
    t_ble = threading.Thread(target=start_ble_thread, daemon=True)
    t_ble.start()

    stm32 = STM32Interface()
    t_stm = threading.Thread(target=stm32.update_loop, daemon=True)
    t_stm.start()

    # --- 视觉初始化 ---
    cap = cv2.VideoCapture(0)
    cap.set(3, 640); cap.set(4, 480)
    
    radar = RadarSystem()
    master_encoding = None
    tracker = None
    
    print("🤖 Robot Online. Waiting for Signal...")

    while system_state["running"]:
        ret, frame = cap.read()
        if not ret: break
        
        # 状态机逻辑
        mode = system_state["mode"]
        current_rssi = system_state["rssi"]
        dist = system_state["distance"]

        # === 状态 1: 等待信号 (WAITING) ===
        if mode == "WAITING":
            stm32.move(0, 0)
            status_text = f"WAITING... RSSI: {current_rssi} dBm"
            
            if current_rssi > RSSI_THRESHOLD:
                print("🚨 Signal Detected! Starting Sweep...")
                system_state["mode"] = "SCANNING"
                radar.start()

        # === 状态 2: 雷达扫描 (SCANNING) ===
        elif mode == "SCANNING":
            elapsed = time.time() - radar.start_time
            status_text = f"SCANNING... {SCAN_DURATION - elapsed:.1f}s"
            
            # 机器人自旋 (左转)
            stm32.move(-SCAN_SPEED_PWM, SCAN_SPEED_PWM)
            
            # 记录数据
            radar.record(frame, current_rssi)

            if elapsed > SCAN_DURATION:
                stm32.move(0, 0) # 停止
                master_encoding = radar.analyze()
                
                if master_encoding is not None:
                    system_state["mode"] = "LOCKED"
                    # 初始化 KCF 追踪器
                    # 既然已经认主，我们假设主人现在就在画面里 (虽然可能不在正中间)
                    # 为了简化，LOCKED 状态初期先用人脸识别找一次，然后把框给 Tracker
                    tracker_init_needed = True 
                else:
                    print("❌ Lock Failed. Retrying...")
                    system_state["mode"] = "WAITING"
                    # 这里可以加个逻辑：如果失败，反向转一点，或者休息一下

        # === 状态 3: 锁定跟随 (LOCKED) ===
        elif mode == "LOCKED":
            status_text = "LOCKED: Following Master"
            
            # 避障最高优先级
            if dist < SAFE_DISTANCE:
                stm32.move(0, 0)
                status_text = "OBSTACLE DETECTED!"
                cv2.putText(frame, "OBSTACLE", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
            else:
                # 视觉处理
                small_frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
                rgb_small = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
                
                # 每一帧都尝试找 Master 的脸 (为了最稳，不用 KCF 了，Pi 5 跑 face_recognition 小图应该有 5-8fps)
                # 如果追求速度，可以像之前那样加 KCF 混合逻辑
                face_locs = face_recognition.face_locations(rgb_small)
                face_encs = face_recognition.face_encodings(rgb_small, face_locs)
                
                target_box = None
                for (top, right, bottom, left), enc in zip(face_locs, face_encs):
                    matches = face_recognition.compare_faces([master_encoding], enc, tolerance=0.5)
                    if True in matches:
                        # 找到主人
                        target_box = (left*4, right*4) # 还原 X 坐标用于计算中心
                        # 画框
                        cv2.rectangle(frame, (left*4, top*4), (right*4, bottom*4), (0, 255, 0), 2)
                        break
                
                if target_box:
                    # PID 控制
                    left_x, right_x = target_box
                    face_cx = (left_x + right_x) // 2
                    face_width = right_x - left_x
                    
                    error = face_cx - 320
                    turn = int(error * Kp)
                    
                    # 距离控制 (基于脸的大小)
                    throttle = 0
                    if face_width < 100: # 脸太小(太远) -> 追
                        throttle = FOLLOW_SPEED_PWM
                    elif face_width > 150: # 脸太大(太近) -> 停
                        throttle = 0
                    
                    stm32.move(throttle + turn, throttle - turn)
                else:
                    # 丢失目标 -> 停下或原地搜索
                    stm32.move(0, 0)
                    status_text = "Master Lost..."

        # 画面显示
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"Dist: {dist}cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        cv2.imshow("Robot View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            system_state["running"] = False
            break

    # 退出清理
    stm32.move(0, 0)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()