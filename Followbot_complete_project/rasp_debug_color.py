import asyncio
import cv2
import threading
import time
import numpy as np
from bleak import BleakScanner
from gpiozero import OutputDevice, PWMOutputDevice, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory
from ultralytics import YOLO

# ================= 1. 配置区域 (Configuration) =================
IP_CAMERA_URL = "http://172.26.33.3/video.mjpg" 

# 核心参数
SKIP_FRAMES = 3           
RSSI_TRIGGER = -85        
SAFE_DIST_STOP = 15.0     
LOST_TIMEOUT = 5.0        

# 动力参数
FOLLOW_SPEED = 0.50       
BACKUP_SPEED = -0.40      
Kp_turn = 0.001          
TURN_DEAD_ZONE = 40       

# [优化参数]
# 倒车时间补偿系数 (1.1 表示多退 10% 的时间，防止退不到位)
REVERSE_TIME_GAIN = 1.1      
# 录制最小速度阈值 (平均速度小于此值视为静止，不录制)
RECORD_MIN_SPEED = 0.1 

# 近距离锁头参数
STOP_TURN_HEIGHT = 0.75   

# 距离保持参数
TARGET_HEIGHT_MIN = 0.55  
TARGET_HEIGHT_MAX = 0.75  

# 硬件偏差修正 (请根据你的硬件实际情况微调，确保给相同数值时走直线)
LEFT_CORRECTION = 1.0     
RIGHT_CORRECTION = 1.0   

# 追踪参数
IOU_THRESHOLD = 0.3       
COLOR_MATCH_THRESH = 0.50 
MEMORY_RATE = 0.1         

# ================= 2. 硬件层 (Hardware Layer) =================
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

# ================= 3. 视觉特征核心 (Vision Features) =================
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

# ================= 4. 视频流与后台 (System Threads) =================
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

# ================= 5. 主程序 (Main Loop) =================
def main():
    threading.Thread(target=start_ble_bg, daemon=True).start()
    print("Loading YOLOv8n...")
    model = YOLO("yolov8n.pt") 
    feat_mgr = FeatureManager()
    
    print(f"Connecting Cam: {IP_CAMERA_URL}")
    cam = IPCameraStream(IP_CAMERA_URL)
    time.sleep(1.0)
    
    # 全局变量
    candidates_queue = []
    curr_cand_idx = 0
    action_timer = 0
    rssi_samples = []
    
    # === 路径记忆变量 (修改版) ===
    # 格式改为: [(avg_speed, duration), ...] 
    # 只记录前进速度分量，丢弃转向分量
    path_memory = []       
    last_loop_time = 0     
    current_avg_speed = 0  # 当前帧的平均前进速度
    # ==========================
    
    last_seen_time = 0 
    frame_count = 0
    last_target_box = None
    last_target_coords = None
    
    last_mode = system_state["mode"] 
    print(f"=== System Ready. Smart Return Logic (Straight-Line Only) Enabled. ===")

    try:
        while system_state["running"]:
            current_mode = system_state["mode"]
            
            # 模式切换检测
            if current_mode != last_mode:
                print(f" >> MODE SWITCH: {last_mode} -> {current_mode}")
                
                # 如果是刚进入 APPROACHING，重置记忆
                if current_mode == "APPROACHING":
                    path_memory = []
                    last_loop_time = time.time()
                    current_avg_speed = 0
                    print(">> Path memory reset for straight-line recording.")
                
                last_mode = current_mode

            frame = cam.read()
            if frame is None: continue
            h, w, _ = frame.shape 

            cv2.putText(frame, f"MODE: {current_mode}", (10, 30), 0, 0.8, (0, 255, 255), 2)

            # 避障 (STANDBY/MEASURING/RETURNING 不避障)
            if dist_cm := (sensor.distance * 100 if sensor.distance else 999): pass
            if dist_cm < SAFE_DIST_STOP and system_state["mode"] not in ["STANDBY", "MEASURING", "RETURNING"]:
                robot_stop()
                cv2.putText(frame, "OBSTACLE!", (w//2-100, h//2), 0, 1.2, (0,0,255), 3)
                cv2.imshow("Robot View", frame) 
                cv2.waitKey(1)
                continue

            should_detect = (frame_count % SKIP_FRAMES == 0)
            
            # ================= 状态机逻辑 =================
            
            if current_mode == "STANDBY":
                robot_stop()
                cv2.putText(frame, "Wait for BLE Signal...", (10, 60), 0, 0.6, (200,200,200), 1)
                cv2.putText(frame, f"RSSI: {system_state['rssi']}", (10, 80), 0, 0.6, (200,200,200), 1)
                
                if system_state['rssi'] > RSSI_TRIGGER:
                    print(">> Triggered! Scanning candidates...")
                    system_state["mode"] = "INIT_SCAN"

            elif current_mode == "INIT_SCAN":
                robot_stop()
                results = model(frame, classes=[0], verbose=False)
                candidates_queue = []
                
                if results[0].boxes:
                    for box in results[0].boxes:
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        area = (xyxy[2]-xyxy[0]) * (xyxy[3]-xyxy[1])
                        if area < 5000: continue
                        feats = feat_mgr.extract(frame, xyxy)
                        if feats is not None:
                            candidates_queue.append({'features': feats, 'avg_rssi': -100})
                    
                    if len(candidates_queue) > 0:
                        print(f">> Found {len(candidates_queue)} candidates.")
                        curr_cand_idx = 0
                        system_state["mode"] = "APPROACHING"
                        last_target_box = None
                        last_target_coords = None
                        last_seen_time = time.time()
                    else:
                        print(">> No humans found. Retrying...")
                else:
                    time.sleep(0.5)

            elif current_mode == "APPROACHING":
                # === [路径记忆 - 仅记录直线分量] ===
                now = time.time()
                dt = now - last_loop_time
                last_loop_time = now
                
                # 只有当平均速度大于阈值时才记录 (忽略静止)
                # 假设 Approach 基本都是往前走，忽略后退操作
                if current_avg_speed > RECORD_MIN_SPEED:
                    path_memory.append((current_avg_speed, dt))
                # =================================

                curr_cand = candidates_queue[curr_cand_idx]
                if should_detect:
                    results = model(frame, classes=[0], verbose=False, imgsz=320)
                    best_box_coords = None
                    match_type = "NONE"
                    if results[0].boxes:
                        detected_boxes = [b.xyxy[0].cpu().numpy().astype(int) for b in results[0].boxes]
                        # IOU First
                        if last_target_coords is not None:
                            best_iou = 0
                            for box in detected_boxes:
                                iou = calculate_iou(last_target_coords, box)
                                if iou > IOU_THRESHOLD and iou > best_iou:
                                    best_iou = iou
                                    best_box_coords = box
                            if best_box_coords is not None: match_type = f"IOU {best_iou:.2f}"
                        # Color Second
                        if best_box_coords is None:
                            min_dist = 999.0
                            for box in detected_boxes:
                                curr_feats = feat_mgr.extract(frame, box)
                                dist = feat_mgr.compare(curr_cand['features'], curr_feats)
                                if dist < COLOR_MATCH_THRESH and dist < min_dist:
                                    min_dist = dist
                                    best_box_coords = box
                                    match_type = f"COL {dist:.2f}"
                        if best_box_coords is not None:
                            last_target_coords = best_box_coords
                            x1, y1, x2, y2 = best_box_coords
                            last_target_box = ((x1+x2)//2, (y1+y2)//2, x2-x1, y2-y1)
                            last_seen_time = time.time()
                            new_feats = feat_mgr.extract(frame, best_box_coords)
                            if new_feats is not None:
                                curr_cand['features'] = cv2.addWeighted(curr_cand['features'], 1.0 - MEMORY_RATE, new_feats, MEMORY_RATE, 0)

                # Approach Control
                time_since_lost = time.time() - last_seen_time
                if last_target_box and time_since_lost < 2.0:
                    cx, cy, obj_w, obj_h = last_target_box
                    cv2.rectangle(frame, (cx-obj_w//2, cy-obj_h//2), (cx+obj_w//2, cy+obj_h//2), (255,0,255), 2)
                    cv2.putText(frame, f"Inv #{curr_cand_idx} [{match_type}]", (cx, cy-10), 0, 0.6, (255,0,255), 2)
                    
                    err_x = cx - w // 2
                    turn_cmd = 0 if abs(err_x) < TURN_DEAD_ZONE else err_x * Kp_turn
                    h_ratio = obj_h / h

                    if h_ratio > STOP_TURN_HEIGHT:
                        turn_cmd = 0
                        cv2.putText(frame, "[LOCK] TOO CLOSE", (cx, cy+20), 0, 0.6, (0,0,255), 2)

                    if h_ratio > 0.65: 
                        robot_stop()
                        current_avg_speed = 0 # 记录为0
                        system_state["mode"] = "MEASURING"
                        action_timer = time.time()
                        rssi_samples = []
                    else:
                        fwd_cmd = FOLLOW_SPEED 
                        l_val = fwd_cmd + turn_cmd
                        r_val = fwd_cmd - turn_cmd
                        robot_move(l_val, r_val)
                        
                        # [关键修改]
                        # 计算平均前进速度 (提取直线分量，忽略转向差值)
                        # 例如: 左轮0.6, 右轮0.4 -> 平均0.5。记录0.5。
                        # 倒车时: 左轮-0.5, 右轮-0.5。
                        avg_val = (l_val + r_val) / 2.0
                        current_avg_speed = avg_val

                else:
                    robot_stop()
                    current_avg_speed = 0
                    cv2.putText(frame, f"LOST... {time_since_lost:.1f}s", (10, 60), 0, 0.8, (0,0,255), 2)
                    if time_since_lost > LOST_TIMEOUT:
                        print(">> Lost candidate timeout.")
                        system_state["mode"] = "STANDBY"

            elif current_mode == "MEASURING":
                robot_stop()
                rssi_samples.append(system_state["rssi"])
                elapsed = time.time() - action_timer
                cv2.putText(frame, f"Measuring RSSI... {3.0-elapsed:.1f}", (10, 60), 0, 1, (255,255,0), 2)
                
                if elapsed > 3.0:
                    avg = sum(rssi_samples)/len(rssi_samples) if rssi_samples else -100
                    candidates_queue[curr_cand_idx]['avg_rssi'] = avg
                    print(f">> Candidate #{curr_cand_idx} Final RSSI: {avg}")

                    # 只要还有候选人，强制倒车
                    if curr_cand_idx + 1 < len(candidates_queue):
                        print(f">> More candidates left. Returning to start...")
                        system_state["mode"] = "RETURNING"
                    else:
                        print(">> All candidates measured. Deciding...")
                        system_state["mode"] = "DECIDING"

            elif current_mode == "RETURNING":
                # === [直线倒车 - 回放逻辑] ===
                print(f">> Start Straight Reverse. Steps: {len(path_memory)}")
                
                # 倒序回放
                while len(path_memory) > 0:
                    # 取出记录的 (平均速度, 持续时间)
                    recorded_speed, duration = path_memory.pop()
                    
                    # 强制直线后退：左右轮速度完全一致，且取反
                    reverse_speed = -recorded_speed
                    robot_move(reverse_speed, reverse_speed)
                    
                    cv2.putText(frame, f"Reversing Straight... Steps: {len(path_memory)}", (10, 60), 0, 0.8, (0,0,255), 2)
                    cv2.imshow("Robot View", frame)
                    
                    # 时间补偿 (乘以系数)
                    comp_duration = duration * REVERSE_TIME_GAIN
                    
                    ms = int(comp_duration * 1000)
                    if ms < 1: ms = 1
                    cv2.waitKey(ms) 

                robot_stop()
                print(">> Returned to start point (Straight).")
                
                curr_cand_idx += 1
                if curr_cand_idx < len(candidates_queue):
                    system_state["mode"] = "APPROACHING"
                    last_target_box = None
                    last_target_coords = None
                    last_seen_time = time.time()
                else:
                    system_state["mode"] = "DECIDING"

            elif current_mode == "DECIDING":
                robot_stop()
                if not candidates_queue:
                    system_state["mode"] = "STANDBY"
                    continue
                
                best_c = max(candidates_queue, key=lambda x: x['avg_rssi'])
                print(f">> Winner RSSI: {best_c['avg_rssi']}")
                
                if best_c['avg_rssi'] > -90:
                    system_state["target_candidate"] = best_c
                    system_state["mode"] = "FOLLOWING"
                    last_target_box = None
                    last_target_coords = None
                    last_seen_time = time.time()
                else:
                    print(">> RSSI too low.")
                    system_state["mode"] = "STANDBY"

            elif current_mode == "FOLLOWING":
                target = system_state["target_candidate"]
                if should_detect:
                    results = model(frame, classes=[0], verbose=False, imgsz=320)
                    best_box_coords = None
                    match_type = "NONE"
                    if results[0].boxes:
                        detected_boxes = [b.xyxy[0].cpu().numpy().astype(int) for b in results[0].boxes]
                        if last_target_coords is not None:
                            best_iou = 0
                            for box in detected_boxes:
                                iou = calculate_iou(last_target_coords, box)
                                if iou > IOU_THRESHOLD and iou > best_iou:
                                    best_iou = iou
                                    best_box_coords = box
                            if best_box_coords is not None: match_type = f"IOU {best_iou:.2f}"
                        if best_box_coords is None:
                            min_dist = 999.0
                            for box in detected_boxes:
                                curr_feats = feat_mgr.extract(frame, box)
                                dist = feat_mgr.compare(target['features'], curr_feats)
                                if dist < COLOR_MATCH_THRESH and dist < min_dist:
                                    min_dist = dist
                                    best_box_coords = box
                                    match_type = f"COL {dist:.2f}"
                        if best_box_coords is not None:
                            last_target_coords = best_box_coords
                            x1, y1, x2, y2 = best_box_coords
                            last_target_box = ((x1+x2)//2, (y1+y2)//2, x2-x1, y2-y1)
                            last_seen_time = time.time()
                            new_feats = feat_mgr.extract(frame, best_box_coords)
                            if new_feats is not None:
                                target['features'] = cv2.addWeighted(target['features'], 0.9, new_feats, 0.1, 0)

                if last_target_box and (time.time() - last_seen_time < 2.0):
                    cx, cy, obj_w, obj_h = last_target_box
                    cv2.rectangle(frame, (cx-obj_w//2, cy-obj_h//2), (cx+obj_w//2, cy+obj_h//2), (0,255,0), 3)
                    cv2.putText(frame, f"MASTER [{match_type}]", (cx, cy-20), 0, 0.8, (0,255,0), 2)
                    
                    err_x = cx - w // 2
                    h_ratio = obj_h / h
                    turn_cmd = 0 if abs(err_x) < TURN_DEAD_ZONE else err_x * Kp_turn
                    
                    if h_ratio > STOP_TURN_HEIGHT:
                        turn_cmd = 0
                        cv2.putText(frame, "[LOCK] NO TURN", (cx, cy+40), 0, 0.6, (0,0,255), 2)

                    if h_ratio < TARGET_HEIGHT_MIN: 
                        fwd_cmd = FOLLOW_SPEED
                        cv2.putText(frame, "CHASING", (10, 60), 0, 0.8, (0,255,0), 2)
                        robot_move(fwd_cmd + turn_cmd, fwd_cmd - turn_cmd)
                    elif h_ratio > TARGET_HEIGHT_MAX: 
                        cv2.putText(frame, "BACKING", (10, 60), 0, 0.8, (0,0,255), 2)
                        robot_move(BACKUP_SPEED, BACKUP_SPEED) 
                    else:
                        cv2.putText(frame, "HOLDING", (10, 60), 0, 0.8, (255,255,0), 2)
                        robot_stop() 
                else:
                    robot_stop()
                    elapsed_lost = time.time() - last_seen_time
                    cv2.putText(frame, f"LOST... {elapsed_lost:.1f}s", (10, 80), 0, 1, (0,0,255), 2)
                    if elapsed_lost > LOST_TIMEOUT:
                        print(">> Target lost completely.")
                        system_state["mode"] = "STANDBY"
                        system_state["target_candidate"] = None 

            cv2.imshow("Robot View", frame)
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