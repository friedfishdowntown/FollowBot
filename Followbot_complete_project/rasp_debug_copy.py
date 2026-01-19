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

# 跳帧参数
SKIP_FRAMES = 3           # 每 3 帧侦测一次 (降低延迟的关键)

# YOLO 推理尺寸和置信度
INFERENCE_SIZE = 320      # 速度与精度的平衡点
CONFIDENCE_THRESHOLD = 0.5  # 置信度阈值

# 动力与阈值
FOLLOW_SPEED = 0.70       
BACKUP_SPEED = -0.65      
SEARCH_TURN_SPEED = 0.6   
RSSI_TRIGGER = -85        
SAFE_DIST_STOP = 15.0     

# PID
Kp_turn = 0.0025          

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
        # 死区补偿
        if 0 < abs(speed) < 0.2: speed = 0.2 if speed > 0 else -0.2
        
        if speed > 0: self.in1.on(); self.in2.off()
        elif speed < 0: self.in1.off(); self.in2.on()
        else: self.in1.off(); self.in2.off()
        self.pwm.value = abs(speed)

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

motor_left = TB6612_Motor(12, 5, 6)
motor_right = TB6612_Motor(13, 22, 27)

def robot_move(l, r): motor_left.drive(l); motor_right.drive(r)
def robot_stop(): motor_left.stop(); motor_right.stop()

# ================= 3. 视觉特征核心 =================
class FeatureManager:
    def extract(self, frame, box):
        """从 YOLO 框中提取颜色特征"""
        x1, y1, x2, y2 = map(int, box)
        h, w, _ = frame.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None
        
        rh, rw, _ = roi.shape
        # 取中间区域作为衣服颜色，避开背景
        shirt_roi = roi[int(rh*0.15):int(rh*0.45), int(rw*0.2):int(rw*0.8)]
        
        features = {}
        if shirt_roi.size > 0:
            hsv = cv2.cvtColor(shirt_roi, cv2.COLOR_BGR2HSV)
            # 计算 HSV 直方图均值
            mean_hsv = np.mean(hsv, axis=(0, 1))
            features['shirt_hsv'] = mean_hsv
        return features

    def compare(self, features_a, features_b):
        """对比两个特征的相似度 (距离越小越相似)"""
        if not features_a or not features_b: return 999.0
        if 'shirt_hsv' not in features_a or 'shirt_hsv' not in features_b: return 999.0
        
        # 计算欧氏距离
        dist = np.linalg.norm(features_a['shirt_hsv'] - features_b['shirt_hsv'])
        return dist

# ================= 4. 视频流与后台 =================
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
    "running": True,
    "target_candidate": None 
}

# 蓝牙
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
    t = threading.Thread(target=start_ble_bg, daemon=True)
    t.start()
    
    print("Loading YOLOv8n (Skip-Frame Optimized)...")
    model = YOLO("yolov8n.pt") 
    feat_mgr = FeatureManager()
    
    print(f"Connecting Cam: {IP_CAMERA_URL}")
    cam = IPCameraStream(IP_CAMERA_URL)
    time.sleep(1.0)
    
    # 状态变量
    candidates_queue = []
    curr_cand_idx = 0
    action_timer = 0
    rssi_samples = []
    
    # 跳帧缓存
    frame_count = 0
    last_target_box = None  # 上一次锁定的目标框 (cx, cy, w, h)
    last_motor_cmd = (0, 0) # 上一次的电机指令
    
    print(f"System Ready. Detection every {SKIP_FRAMES} frames.")

    try:
        while system_state["running"]:
            frame = cam.read()
            if frame is None: continue
            
            # 避障 (每一帧都检查，不跳帧，保证安全)
            dist_cm = sensor.distance * 100 if sensor.distance else 999
            if dist_cm < SAFE_DIST_STOP and system_state["mode"] not in ["STANDBY", "MEASURING"]:
                robot_stop()
                cv2.putText(frame, "OBSTACLE!", (100, 100), 0, 1, (0,0,255), 3)
                cv2.imshow("Robot View", frame)
                cv2.waitKey(1)
                continue

            mode = system_state["mode"]
            h, w, _ = frame.shape
            
            # 决定是否跑 YOLO
            should_detect = (frame_count % SKIP_FRAMES == 0)
            
            # === 状态机 ===
            
            # 1. 待机
            if mode == "STANDBY":
                robot_stop()
                last_motor_cmd = (0, 0)
                if system_state['rssi'] > RSSI_TRIGGER:
                    print(">> Triggered! Scanning...")
                    system_state["mode"] = "INIT_SCAN"

            # 2. 初始化扫描 (不跳帧，跑一次完整的)
            elif mode == "INIT_SCAN":
                robot_stop()
                # 使用 model.predict 方法，带置信度阈值
                results = model.predict(frame, classes=0, imgsz=INFERENCE_SIZE, verbose=False, conf=CONFIDENCE_THRESHOLD)
                candidates_queue = []
                
                if len(results) > 0 and results[0].boxes is not None:
                    for box in results[0].boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        # 提取特征
                        feats = feat_mgr.extract(frame, xyxy)
                        if feats:
                            candidates_queue.append({
                                'features': feats,
                                'avg_rssi': -100
                            })
                    
                    if len(candidates_queue) > 0:
                        print(f">> Candidates found: {len(candidates_queue)}")
                        curr_cand_idx = 0
                        system_state["mode"] = "APPROACHING"
                        last_target_box = None
                    else:
                        print(">> No candidates. Retry.")
                else:
                    time.sleep(0.5)

            # 3. 靠近 (跳帧逻辑生效)
            elif mode == "APPROACHING":
                curr_cand = candidates_queue[curr_cand_idx]
                
                # --- A. 侦测帧 ---
                if should_detect:
                    # 使用 model.predict 方法找人
                    results = model.predict(frame, classes=0, imgsz=INFERENCE_SIZE, verbose=False, conf=CONFIDENCE_THRESHOLD)
                    best_match_box = None
                    min_dist = 999.0
                    
                    if len(results) > 0 and results[0].boxes is not None:
                        for box in results[0].boxes:
                            xyxy = box.xyxy[0].cpu().numpy()
                            # 提取当前这个人的特征
                            curr_feats = feat_mgr.extract(frame, xyxy)
                            # 和目标对比
                            dist = feat_mgr.compare(curr_cand['features'], curr_feats)
                            
                            # 阈值判断 (颜色差异小于 40 算匹配)
                            if dist < 40 and dist < min_dist:
                                min_dist = dist
                                x1, y1, x2, y2 = map(int, xyxy)
                                best_match_box = ((x1+x2)//2, (y1+y2)//2, x2-x1, y2-y1)

                    last_target_box = best_match_box
                
                # --- B. 控制逻辑 (每一帧都跑) ---
                if last_target_box:
                    cx, cy, obj_w, obj_h = last_target_box
                    
                    # 画框
                    cv2.rectangle(frame, (cx-obj_w//2, cy-obj_h//2), (cx+obj_w//2, cy+obj_h//2), (255,0,255), 2)
                    cv2.putText(frame, f"Inv #{curr_cand_idx}", (cx, cy), 0, 0.7, (255,0,255), 2)
                    
                    # 判断距离
                    if (obj_h / h) > 0.65: # 靠近到 65% 高度
                        robot_stop()
                        last_motor_cmd = (0, 0)
                        system_state["mode"] = "MEASURING"
                        action_timer = time.time()
                        rssi_samples = []
                    else:
                        # PID
                        err = cx - w//2
                        turn = err * Kp_turn
                        l_spd = FOLLOW_SPEED + turn
                        r_spd = FOLLOW_SPEED - turn
                        robot_move(l_spd, r_spd)
                        last_motor_cmd = (l_spd, r_spd)
                else:
                    # 如果这几帧都没看到目标 -> 搜寻模式
                    # 每一帧都执行旋转
                    robot_move(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED)
                    cv2.putText(frame, "SEARCHING...", (10, 60), 0, 0.7, (0,0,255), 2)

            # 4. 测量 (无视觉移动)
            elif mode == "MEASURING":
                robot_stop()
                rssi_samples.append(system_state["rssi"])
                elapsed = time.time() - action_timer
                cv2.putText(frame, f"Measuring... {3.0-elapsed:.1f}", (10, 60), 0, 1, (255,255,0), 2)
                
                if elapsed > 3.0:
                    avg = sum(rssi_samples)/len(rssi_samples) if rssi_samples else -100
                    candidates_queue[curr_cand_idx]['avg_rssi'] = avg
                    system_state["mode"] = "RETURNING"
                    action_timer = time.time()

            # 5. 后退 (盲操作)
            elif mode == "RETURNING":
                robot_move(BACKUP_SPEED, BACKUP_SPEED)
                elapsed = time.time() - action_timer
                if elapsed > 2.5:
                    robot_stop()
                    curr_cand_idx += 1
                    if curr_cand_idx < len(candidates_queue):
                        system_state["mode"] = "APPROACHING"
                        last_target_box = None # 重置目标
                    else:
                        system_state["mode"] = "DECIDING"

            # 6. 决策
            elif mode == "DECIDING":
                robot_stop()
                best_c = max(candidates_queue, key=lambda x: x['avg_rssi'])
                print(f"Winner RSSI: {best_c['avg_rssi']}")
                if best_c['avg_rssi'] > -90:
                    system_state["target_candidate"] = best_c
                    system_state["mode"] = "FOLLOWING"
                    last_target_box = None
                else:
                    system_state["mode"] = "STANDBY"

            # 7. 跟随 (跳帧逻辑生效)
            elif mode == "FOLLOWING":
                target = system_state["target_candidate"]
                
                # --- A. 侦测 ---
                if should_detect:
                    results = model.predict(frame, classes=0, imgsz=INFERENCE_SIZE, verbose=False, conf=CONFIDENCE_THRESHOLD)
                    best_match_box = None
                    min_dist = 999.0
                    
                    if len(results) > 0 and results[0].boxes is not None:
                        for box in results[0].boxes:
                            xyxy = box.xyxy[0].cpu().numpy()
                            curr_feats = feat_mgr.extract(frame, xyxy)
                            dist = feat_mgr.compare(target['features'], curr_feats)
                            
                            if dist < 40 and dist < min_dist:
                                min_dist = dist
                                x1, y1, x2, y2 = map(int, xyxy)
                                best_match_box = ((x1+x2)//2, (y1+y2)//2, x2-x1, y2-y1)
                    
                    last_target_box = best_match_box
                
                # --- B. 控制 ---
                if last_target_box:
                    cx, cy, obj_w, obj_h = last_target_box
                    cv2.rectangle(frame, (cx-obj_w//2, cy-obj_h//2), (cx+obj_w//2, cy+obj_h//2), (0,255,0), 3)
                    cv2.putText(frame, "MASTER", (cx, cy-20), 0, 1, (0,255,0), 2)
                    
                    err = cx - w//2
                    turn = err * Kp_turn
                    
                    fwd = 0
                    if obj_w < 180: fwd = FOLLOW_SPEED 
                    elif obj_w > 220: fwd = BACKUP_SPEED * 0.5 
                    
                    robot_move(fwd+turn, fwd-turn)
                else:
                    robot_stop()
                    cv2.putText(frame, "LOST MASTER", (10, 60), 0, 1, (0,0,255), 2)

            cv2.imshow("Robot Skip-Frame View", frame)
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