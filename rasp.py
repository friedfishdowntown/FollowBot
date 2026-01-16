import asyncio
import cv2
import face_recognition
import threading
import time
import numpy as np
from bleak import BleakScanner
from gpiozero import Motor, DistanceSensor, OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# ================= 硬件配置 =================
# 指定 Pin Factory，防止 Pi 5 兼容性问题
factory = LGPIOFactory()

# --- 超声波配置 ---
# echo=GPIO24, trigger=GPIO23 (请确保按照上面说的接了分压电阻！)
sensor = DistanceSensor(echo=24, trigger=23, max_distance=4, pin_factory=factory)

# --- 电机配置 (TB6612) ---
# TB6612 需要 3 个引脚控制一个电机: PWM, IN1, IN2
class TB6612_Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, pin_factory=factory)
        self.in1 = OutputDevice(in1_pin, pin_factory=factory)
        self.in2 = OutputDevice(in2_pin, pin_factory=factory)
    
    def drive(self, speed):
        """
        speed范围: -1.0 到 1.0 (负数后退，正数前进，0停止)
        """
        # 限制范围
        speed = max(min(speed, 1.0), -1.0)
        
        # 设置方向
        if speed > 0:
            self.in1.on()
            self.in2.off()
        elif speed < 0:
            self.in1.off()
            self.in2.on()
        else:
            self.in1.off()
            self.in2.off()
        
        # 设置速度 (PWM占空比)
        self.pwm.value = abs(speed)

# 初始化左右电机
# 左轮: PWM=12, IN1=5, IN2=6
motor_left = TB6612_Motor(12, 5, 6)
# 右轮: PWM=13, IN1=22, IN2=27
motor_right = TB6612_Motor(13, 22, 27)


# ================= 参数定义 =================
TAG_NAME = "FollowMe_Tag"
SCAN_DURATION = 12.0
# 速度调整为 0.0 - 1.0 之间
SCAN_SPEED = 0.3      
FOLLOW_SPEED = 0.5
RSSI_THRESHOLD = -80
SAFE_DISTANCE_CM = 30.0
Kp = 0.002  # PID参数需要重新调，因为现在输出是 0-1.0，之前的error是像素(几百)

# ================= 全局状态 =================
system_state = {
    "rssi": -100,
    "mode": "WAITING",
    "running": True
}

# ================= 1. 蓝牙扫描 (不变) =================
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

# ================= 2. 硬件控制封装 =================
def robot_move(left_speed, right_speed):
    motor_left.drive(left_speed)
    motor_right.drive(right_speed)

def get_distance():
    # gpiozero 返回的是米，我们要转换成厘米
    # 如果读取失败或无限远，它可能返回 None 或 1.0
    d = sensor.distance
    if d is None:
        return 999.0
    return d * 100.0

# ================= 3. 雷达与主逻辑 (保留逻辑，替换硬件调用) =================
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
        small = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
        self.visual_log.append({'t': now, 'img': small})

    def analyze(self):
        # ... (保持原来的逻辑不变) ...
        print("Analyzing Scan Data...")
        if not self.rssi_log: return None
        rssi_vals = [x['rssi'] for x in self.rssi_log]
        if len(rssi_vals) < 5: return None
        smoothed = np.convolve(rssi_vals, np.ones(5)/5, mode='valid')
        peak_idx = np.argmax(smoothed) + 2
        peak_time = self.rssi_log[peak_idx]['t']
        
        # 找对应时间的图片
        best_frame_data = min(self.visual_log, key=lambda x: abs(x['t'] - peak_time))
        frame = best_frame_data['img']
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        boxes = face_recognition.face_locations(rgb)
        if not boxes: return None
        
        h, w, _ = frame.shape
        center_x = w // 2
        best_box = min(boxes, key=lambda b: abs((b[3]+b[1])/2 - center_x))
        encoding = face_recognition.face_encodings(rgb, [best_box])[0]
        return encoding

def main():
    # 启动蓝牙线程
    t_ble = threading.Thread(target=start_ble_thread, daemon=True)
    t_ble.start()

    cap = cv2.VideoCapture(0)
    cap.set(3, 640); cap.set(4, 480)
    
    radar = RadarSystem()
    master_encoding = None
    
    print("System Pi-Only Ready.")

    try:
        while system_state["running"]:
            ret, frame = cap.read()
            if not ret: break
            
            mode = system_state["mode"]
            current_rssi = system_state["rssi"]
            
            # 直接读取距离
            dist_cm = get_distance()

            status_text = ""

            # === WAITING ===
            if mode == "WAITING":
                robot_move(0, 0)
                status_text = f"WAIT: {current_rssi}dBm"
                if current_rssi > RSSI_THRESHOLD:
                    system_state["mode"] = "SCANNING"
                    radar.start()

            # === SCANNING ===
            elif mode == "SCANNING":
                elapsed = time.time() - radar.start_time
                status_text = f"SCAN: {elapsed:.1f}s"
                
                # 原地旋转 (注意方向，左负右正)
                robot_move(-SCAN_SPEED, SCAN_SPEED)
                radar.record(frame, current_rssi)

                if elapsed > SCAN_DURATION:
                    robot_move(0, 0)
                    master_encoding = radar.analyze()
                    if master_encoding is not None:
                        system_state["mode"] = "LOCKED"
                    else:
                        system_state["mode"] = "WAITING"

            # === LOCKED ===
            elif mode == "LOCKED":
                status_text = "LOCKED"
                
                if dist_cm < SAFE_DISTANCE_CM:
                    robot_move(0, 0)
                    status_text = "OBSTACLE!"
                    cv2.putText(frame, "STOP", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
                else:
                    small_frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
                    rgb_small = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
                    
                    face_locs = face_recognition.face_locations(rgb_small)
                    face_encs = face_recognition.face_encodings(rgb_small, face_locs)
                    
                    target_box = None
                    for (top, right, bottom, left), enc in zip(face_locs, face_encs):
                        matches = face_recognition.compare_faces([master_encoding], enc, tolerance=0.5)
                        if True in matches:
                            target_box = (left*4, right*4)
                            break
                    
                    if target_box:
                        l, r = target_box
                        cx = (l + r) // 2
                        width = r - l
                        
                        # PID 计算 (注意 Kp 要很小，因为输出只有 0-1)
                        error = cx - 320
                        turn = error * Kp 
                        
                        throttle = 0
                        if width < 100: throttle = FOLLOW_SPEED
                        elif width > 150: throttle = 0
                        
                        # 混合控制
                        l_out = throttle + turn
                        r_out = throttle - turn
                        robot_move(l_out, r_out)
                    else:
                        robot_move(0, 0)
                        status_text = "Lost..."

            # UI 显示
            cv2.putText(frame, f"{status_text} Dist:{dist_cm:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Main", frame)
            
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
