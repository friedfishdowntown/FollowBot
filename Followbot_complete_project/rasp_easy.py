import asyncio
import cv2
import threading
import time
import numpy as np
from bleak import BleakScanner
from gpiozero import OutputDevice, PWMOutputDevice, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory
from ultralytics import YOLO

# ================= 1. 配置区域 =================
IP_CAMERA_URL = "http://172.26.33.3/video.mjpg" 

# 核心参数
SKIP_FRAMES = 2           
RSSI_TRIGGER = -85        
LOST_TIMEOUT = 5.0        

# 动力参数
FOLLOW_SPEED = 0.50       # 前进速度
BACKUP_SPEED = -0.50      # 后退速度 (可选，本版本策略是太近优先停车)
Kp_turn = 0.001          
TURN_DEAD_ZONE = 40       

# 硬件偏差修正 (根据你的机器人调整)
LEFT_CORRECTION = 1.0     
RIGHT_CORRECTION = 0.90   

# [UPDATED] 距离与停止策略参数
# 逻辑：
# 1. 高度 < STOP_HEIGHT_MIN (0.45): 追 (Forward + Turn)
# 2. 高度 > STOP_HEIGHT_MIN (0.45) 且 < STOP_HEIGHT_MAX (0.60): 停 (Stop + No Turn) -> 完美停车区
# 3. 高度 > STOP_HEIGHT_MAX (0.60): 太近了 (Stop + No Turn) -> 防止贴脸后转向
STOP_HEIGHT_MIN = 0.45    # 目标达到这个大小就开始停车
STOP_HEIGHT_MAX = 0.60    # 超过这个大小视为"贴脸"

# 追踪参数
IOU_THRESHOLD = 0.3       
COLOR_MATCH_THRESH = 0.50 
MEMORY_RATE = 0.1         

# ================= 2. 硬件层 =================
factory = LGPIOFactory()
sensor = DistanceSensor(echo=24, trigger=23, max_distance=4, pin_factory=factory)

class TB6612_Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, pin_factory=factory)
        self.in1 = OutputDevice(in1_pin, pin_factory=factory)
        self.in2 = OutputDevice(in2_pin, pin_factory=factory)
    
    def drive(self, speed):
        speed = max(min(speed, 1.0), -1.0)
        if 0 < abs(speed) < 0.25: speed = 0.25 if speed > 0 else -0.25
        
        if speed > 0: self.in1.on(); self.in2.off()
        elif speed < 0: self.in1.off(); self.in2.on()
        else: self.in1.off(); self.in2.off()
        self.pwm.value = abs(speed)

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

motor_left = TB6612_Motor(12, 5, 6)
motor_right = TB6612_Motor(13, 22, 27)

def robot_move(left_speed, right_speed):
    l_final = left_speed * LEFT_CORRECTION
    r_final = right_speed * RIGHT_CORRECTION
    motor_left.drive(l_final)
    motor_right.drive(r_final)

def robot_stop():
    motor_left.stop()
    motor_right.stop()

# ================= 3. 视觉特征 =================
class FeatureManager:
    def extract(self, frame, box):
        x1, y1, x2, y2 = map(int, box)
        h, w, _ = frame.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None
        rh, rw, _ = roi.shape
        cy, cx = rh // 2, rw // 2
        dy, dx = int(rh * 0.2), int(rw * 0.2)
        center_roi = roi[cy-dy:cy+dy, cx-dx:cx+dx]
        if center_roi.size == 0: return None
        hsv = cv2.cvtColor(center_roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    def compare(self, hist1, hist2):
        if hist1 is None or hist2 is None: return 1.0
        return cv2.compareHist(hist1, hist2, cv2.HISTCMP_BHATTACHARYYA)

def calculate_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return interArea / float(boxAArea + boxBArea - interArea + 1e-6)

# ================= 4. 视频流与蓝牙 =================
class IPCameraStream:
    def __init__(self, src):
        self.stream = cv2.VideoCapture(src)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        threading.Thread(target=self.update, daemon=True).start()
    def update(self):
        while not self.stopped:
            if self.stream.isOpened():
                (grabbed, frame) = self.stream.read()
                if grabbed: self.frame = frame
            else: time.sleep(0.1)
    def read(self): return self.frame
    def release(self): self.stopped = True; self.stream.release()

system_state = {
    "rssi": -100,
    "mode": "STANDBY",
    "running": True
}

async def ble_scanner():
    def callback(d, a):
        if d.name == "XIAO_ESP32": system_state["rssi"] = a.rssi
    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    while system_state["running"]: await asyncio.sleep(1)
    await scanner.stop()

def start_ble_bg():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_scanner())

# ================= 5. 主程序 =================
def main():
    threading.Thread(target=start_ble_bg, daemon=True).start()
    print("Loading YOLOv8n...")
    model = YOLO("yolov8n.pt") 
    feat_mgr = FeatureManager()
    
    print(f"Connecting Cam: {IP_CAMERA_URL}")
    cam = IPCameraStream(IP_CAMERA_URL)
    time.sleep(1.0)
    
    target_features = None
    last_target_coords = None
    last_seen_time = 0
    frame_count = 0

    print("=== System Ready: Auto-Stop & Anti-Spin ===")
    
    try:
        while system_state["running"]:
            frame = cam.read()
            if frame is None: continue
            h, w, _ = frame.shape

            # 避障
            dist_cm = sensor.distance * 100 if sensor.distance else 999
            if dist_cm < 15.0:
                robot_stop()
                cv2.putText(frame, "OBSTACLE!", (50, 50), 0, 1, (0,0,255), 3)
                cv2.imshow("View", frame)
                cv2.waitKey(1)
                continue

            mode = system_state["mode"]
            should_detect = (frame_count % SKIP_FRAMES == 0)

            # --- 状态机 ---
            if mode == "STANDBY":
                robot_stop()
                cv2.putText(frame, f"Wait BLE... RSSI: {system_state['rssi']}", (10, 30), 0, 0.7, (200,200,200), 2)
                if system_state['rssi'] > RSSI_TRIGGER:
                    print(">> Signal Detected! Locking Target...")
                    system_state["mode"] = "LOCKING"

            elif mode == "LOCKING":
                results = model(frame, classes=[0], verbose=False)
                max_area = 0
                best_box = None
                
                if results[0].boxes:
                    for box in results[0].boxes:
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        area = (xyxy[2]-xyxy[0]) * (xyxy[3]-xyxy[1])
                        if area > max_area:
                            max_area = area
                            best_box = xyxy
                
                if best_box is not None:
                    target_features = feat_mgr.extract(frame, best_box)
                    last_target_coords = best_box
                    last_seen_time = time.time()
                    if target_features is not None:
                        system_state["mode"] = "FOLLOWING"
                        print(">> Target LOCKED.")
                    else:
                        print(">> Features failed.")
                else:
                     cv2.putText(frame, "Scanning...", (10, 60), 0, 0.7, (0,165,255), 2)

            elif mode == "FOLLOWING":
                current_box_coords = None
                match_info = "Hold"
                
                # YOLO 检测部分
                if should_detect:
                    results = model(frame, classes=[0], verbose=False, imgsz=320)
                    if results[0].boxes:
                        candidates = []
                        for box in results[0].boxes:
                            candidates.append(box.xyxy[0].cpu().numpy().astype(int))
                        
                        best_iou = 0
                        best_match_idx = -1
                        if last_target_coords is not None:
                            for idx, box in enumerate(candidates):
                                iou = calculate_iou(last_target_coords, box)
                                if iou > best_iou:
                                    best_iou = iou
                                    best_match_idx = idx
                        
                        if best_match_idx != -1 and best_iou > IOU_THRESHOLD:
                            current_box_coords = candidates[best_match_idx]
                            match_info = f"IOU: {best_iou:.2f}"
                        elif target_features is not None:
                            min_dist = 999
                            best_feat_idx = -1
                            for idx, box in enumerate(candidates):
                                feats = feat_mgr.extract(frame, box)
                                d = feat_mgr.compare(target_features, feats)
                                if d < COLOR_MATCH_THRESH and d < min_dist:
                                    min_dist = d
                                    best_feat_idx = idx
                            if best_feat_idx != -1:
                                current_box_coords = candidates[best_feat_idx]
                                match_info = f"COLOR: {min_dist:.2f}"

                        if current_box_coords is not None:
                            last_target_coords = current_box_coords
                            last_seen_time = time.time()
                            new_f = feat_mgr.extract(frame, current_box_coords)
                            if new_f is not None:
                                target_features = cv2.addWeighted(target_features, 1.0-MEMORY_RATE, new_f, MEMORY_RATE, 0)
                    else:
                        current_box_coords = None
                else:
                    current_box_coords = last_target_coords

                # ==========================================
                # [重写] 运动控制逻辑
                # ==========================================
                if current_box_coords is not None:
                    x1, y1, x2, y2 = current_box_coords
                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    obj_h = y2 - y1
                    
                    # 绘制目标框
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    h_ratio = obj_h / h
                    err_x = cx - w // 2
                    
                    # 状态显示文本
                    status_text = "UNKNOWN"

                    # --- 策略 A: 距离太近 (大于 0.45) ---
                    # 策略：完全停车，并且忽视左右转向 (防止贴脸后抖动)
                    if h_ratio > STOP_HEIGHT_MIN:
                        fwd_cmd = 0
                        turn_cmd = 0  # <--- 关键：太近了就不转了，彻底刹车
                        robot_stop()  # 硬件断电
                        
                        if h_ratio > STOP_HEIGHT_MAX:
                            status_text = "TOO CLOSE (STOP)"
                            cv2.putText(frame, "BACK OFF!", (x1, y1-40), 0, 0.6, (0,0,255), 2)
                        else:
                            status_text = "PERFECT DIST (STOP)"
                            cv2.putText(frame, "NICE :)", (x1, y1-40), 0, 0.6, (0,255,0), 2)

                    # --- 策略 B: 距离较远 (小于 0.45) ---
                    # 策略：正常追逐，允许转向
                    else:
                        status_text = "CHASING"
                        fwd_cmd = FOLLOW_SPEED
                        
                        # 死区判断 (防止走直线时微小晃动)
                        if abs(err_x) < TURN_DEAD_ZONE:
                            turn_cmd = 0
                        else:
                            turn_cmd = err_x * Kp_turn
                        
                        robot_move(fwd_cmd + turn_cmd, fwd_cmd - turn_cmd)

                    # Debug UI
                    cv2.putText(frame, f"{status_text} | H:{h_ratio:.2f}", (x1, y1-20), 0, 0.6, (0, 255, 255), 2)
                    
                else:
                    # --- 丢失目标逻辑 ---
                    # [关键修复] 丢失目标时，立即停车，绝对不要旋转
                    robot_stop() 
                    
                    elapsed = time.time() - last_seen_time
                    cv2.putText(frame, f"LOST... {elapsed:.1f}s", (10, 80), 0, 0.8, (0,0,255), 2)
                    
                    if elapsed > LOST_TIMEOUT:
                        print(">> Target lost timeout. Resetting.")
                        system_state["mode"] = "STANDBY"
                        target_features = None
                        last_target_coords = None

            cv2.putText(frame, f"MODE: {mode}", (10, 20), 0, 0.6, (0,255,255), 2)
            cv2.imshow("View", frame)
            frame_count += 1
            if cv2.waitKey(1) == ord('q'): break

    except KeyboardInterrupt:
        pass
    finally:
        robot_stop()
        cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()