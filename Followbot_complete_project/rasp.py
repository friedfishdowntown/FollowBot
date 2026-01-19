import asyncio
import cv2
import threading
import time
import numpy as np
from bleak import BleakScanner
from gpiozero import OutputDevice, PWMOutputDevice, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory
from ultralytics import YOLO

# ================= 1. 配置与参数 =================
# [重要] 填入你的 IP Camera 地址
IP_CAMERA_URL = "http://172.26.33.3/video.mjpg" 

# 阈值设定
RSSI_TRIGGER = -100        
SAFE_DIST_STOP = 15.0     
MATCH_TOLERANCE = 0.6     

# --- [修改点] 动力参数调整 ---
# 负载高时，PWM 必须给大一点才能动起来
# 范围 0.0 - 1.0
FOLLOW_SPEED = 0.70       # 前进基础速度 (原 0.35 -> 0.70)
BACKUP_SPEED = -0.65      # 后退速度 (原 -0.4 -> -0.65)
SEARCH_TURN_SPEED = 0.6   # 原地寻找时的旋转速度 (原 0.3 -> 0.6)

# PID 参数
Kp_turn = 0.0025          # 稍微加大转向力度，防止速度快了转不过弯

# 动作参数
VERIFY_APPROACH_H = 0.65  
VERIFY_MEASURE_TIME = 3.0 
VERIFY_BACKUP_TIME = 2.5  

# ================= 2. 硬件层 =================
factory = LGPIOFactory()
sensor = DistanceSensor(echo=24, trigger=23, max_distance=4, pin_factory=factory)

class TB6612_Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, pin_factory=factory)
        self.in1 = OutputDevice(in1_pin, pin_factory=factory)
        self.in2 = OutputDevice(in2_pin, pin_factory=factory)
    
    def drive(self, speed):
        # 限制在 -1.0 到 1.0 之间
        speed = max(min(speed, 1.0), -1.0)
        
        # 死区补偿 (Deadzone Compensation) - 可选
        # 如果负载真的很重，可能需要给一个最小启动电压，例如 0.2
        if 0 < abs(speed) < 0.2:
            speed = 0.2 if speed > 0 else -0.2

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

# ================= 3. 视觉与特征核心 =================

class FeatureManager:
    """负责提取和匹配颜色特征"""
    def extract(self, frame, box):
        x1, y1, x2, y2 = map(int, box)
        h, w, _ = frame.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None
        
        rh, rw, _ = roi.shape
        shirt_roi = roi[int(rh*0.1):int(rh*0.4), int(rw*0.2):int(rw*0.8)]
        pants_roi = roi[int(rh*0.6):int(rh*0.9), int(rw*0.2):int(rw*0.8)]
        
        features = {}
        if shirt_roi.size > 0:
            hsv_s = cv2.cvtColor(shirt_roi, cv2.COLOR_BGR2HSV)
            mean_s = np.mean(hsv_s, axis=(0, 1))
            features['shirt_range'] = self._get_range(mean_s)
            
        if pants_roi.size > 0:
            hsv_p = cv2.cvtColor(pants_roi, cv2.COLOR_BGR2HSV)
            mean_p = np.mean(hsv_p, axis=(0, 1))
            features['pants_range'] = self._get_range(mean_p)
            
        return features

    def _get_range(self, mean_hsv):
        H, S, V = mean_hsv
        lower = np.array([max(0, H-15), max(20, S-80), max(20, V-80)])
        upper = np.array([min(180, H+15), min(255, S+255), min(255, V+255)])
        return (lower, upper)

    def find_match(self, frame, features):
        if not features: return None
        h, w, _ = frame.shape
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        mask_s = cv2.inRange(hsv, features['shirt_range'][0], features['shirt_range'][1])
        mask_s = cv2.erode(mask_s, None, iterations=2)
        mask_s = cv2.dilate(mask_s, None, iterations=2)
        
        cnts, _ = cv2.findContours(mask_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None
        
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 500: return None
        
        x, y, w_box, h_box = cv2.boundingRect(c)
        return (x + w_box//2, y + h_box//2, w_box, h_box, area)

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

# 全局变量
system_state = {
    "rssi": -100,
    "mode": "STANDBY",  
    "running": True,
    "target_candidate": None 
}

candidates_queue = []
current_candidate_idx = 0
action_timer = 0
rssi_samples = []

# --- [修改点] 修复 RSSI 获取报错 ---
async def ble_scanner():
    # callback(device, advertisement_data)
    def callback(d, a):
   
        # 修正：从 'a' (AdvertisementData) 获取 RSSI
        if d.name == "XIAO_ESP32": 
            system_state["rssi"] = a.rssi
            
    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    while system_state["running"]: await asyncio.sleep(1)
    await scanner.stop()

def start_ble_bg():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_scanner())

# ================= 5. 主逻辑 =================
def main():
    global candidates_queue, current_candidate_idx, action_timer, rssi_samples
    
    t = threading.Thread(target=start_ble_bg, daemon=True)
    t.start()
    
    print("Loading YOLO...")
    model = YOLO("yolov8n.pt")
    feat_mgr = FeatureManager()
    
    print(f"Connecting Cam: {IP_CAMERA_URL}")
    cam = IPCameraStream(IP_CAMERA_URL)
    time.sleep(1.0)
    
    print("System Ready (Turbo Mode). Waiting for RSSI Trigger...")

    try:
        while system_state["running"]:
            frame = cam.read()
            if frame is None: continue
            
            dist_cm = sensor.distance * 100 if sensor.distance else 999
            if dist_cm < SAFE_DIST_STOP and system_state["mode"] not in ["STANDBY", "MEASURING"]:
                robot_stop()
                cv2.putText(frame, "OBSTACLE!", (200, 240), 0, 1, (0,0,255), 3)
                cv2.imshow("Robot View", frame)
                cv2.waitKey(1)
                continue

            mode = system_state["mode"]
            h, w, _ = frame.shape
            
            # === 状态机 ===
            
            if mode == "STANDBY":
                robot_stop()
                cv2.putText(frame, f"WAIT RSSI: {system_state['rssi']}", (10, 30), 0, 1, (0,255,255), 2)
                if system_state['rssi'] > RSSI_TRIGGER:
                    print(">> Triggered! Starting Initialization...")
                    system_state["mode"] = "INIT_SCAN"

            elif mode == "INIT_SCAN":
                robot_stop()
                results = model(frame, classes=[0], verbose=False)
                candidates_queue = []
                
                if results[0].boxes:
                    for box in results[0].boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        feats = feat_mgr.extract(frame, xyxy)
                        if feats:
                            candidates_queue.append({
                                'initial_id': int(box.id[0]) if box.id is not None else 0,
                                'features': feats,
                                'avg_rssi': -100,
                                'status': 'PENDING'
                            })
                    
                    if len(candidates_queue) > 0:
                        print(f">> Found {len(candidates_queue)} candidates. Starting Investigation.")
                        current_candidate_idx = 0
                        system_state["mode"] = "APPROACHING"
                    else:
                        print(">> No extractable features found. Retry.")
                
                if not candidates_queue:
                    time.sleep(0.5)

            elif mode == "APPROACHING":
                curr_cand = candidates_queue[current_candidate_idx]
                match = feat_mgr.find_match(frame, curr_cand['features'])
                
                if match:
                    cx, cy, obj_w, obj_h, area = match
                    cv2.rectangle(frame, (cx-obj_w//2, cy-obj_h//2), (cx+obj_w//2, cy+obj_h//2), (255,0,255), 2)
                    cv2.putText(frame, f"Inv: #{current_candidate_idx}", (cx, cy), 0, 0.7, (255,0,255), 2)
                    
                    if (obj_h / h) > VERIFY_APPROACH_H:
                        print(f">> Reached Candidate #{current_candidate_idx}. Measuring...")
                        robot_stop()
                        system_state["mode"] = "MEASURING"
                        action_timer = time.time()
                        rssi_samples = []
                    else:
                        err = cx - w//2
                        turn = err * Kp_turn
                        # [修改] 使用更大的基础速度
                        robot_move(FOLLOW_SPEED + turn, FOLLOW_SPEED - turn)
                else:
                    # [修改] 寻找时的旋转速度加大
                    robot_move(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED) 
                    cv2.putText(frame, "SEARCHING CANDIDATE...", (10, 60), 0, 0.7, (0,0,255), 2)

            elif mode == "MEASURING":
                robot_stop()
                rssi_samples.append(system_state["rssi"])
                elapsed = time.time() - action_timer
                cv2.putText(frame, f"Measuring... {3.0-elapsed:.1f}", (10, 60), 0, 1, (255,255,0), 2)
                if elapsed > VERIFY_MEASURE_TIME:
                    avg = sum(rssi_samples)/len(rssi_samples) if rssi_samples else -100
                    candidates_queue[current_candidate_idx]['avg_rssi'] = avg
                    candidates_queue[current_candidate_idx]['status'] = 'CHECKED'
                    print(f">> Candidate #{current_candidate_idx} RSSI: {avg:.1f}")
                    system_state["mode"] = "RETURNING"
                    action_timer = time.time()

            elif mode == "RETURNING":
                # [修改] 倒车速度加大
                robot_move(BACKUP_SPEED, BACKUP_SPEED)
                elapsed = time.time() - action_timer
                if elapsed > VERIFY_BACKUP_TIME:
                    robot_stop()
                    current_candidate_idx += 1
                    if current_candidate_idx < len(candidates_queue):
                        print(">> Backing done. Looking for next candidate.")
                        system_state["mode"] = "APPROACHING"
                    else:
                        print(">> All checked. Deciding...")
                        system_state["mode"] = "DECIDING"

            elif mode == "DECIDING":
                robot_stop()
                best_c = max(candidates_queue, key=lambda x: x['avg_rssi'])
                print(f"Winner: Index {candidates_queue.index(best_c)} with RSSI {best_c['avg_rssi']}")
                if best_c['avg_rssi'] > -90:
                    system_state["target_candidate"] = best_c
                    system_state["mode"] = "FOLLOWING"
                else:
                    print("Signal too weak. Restarting.")
                    system_state["mode"] = "STANDBY"

            elif mode == "FOLLOWING":
                target = system_state["target_candidate"]
                match = feat_mgr.find_match(frame, target['features'])
                
                if match:
                    cx, cy, obj_w, obj_h, _ = match
                    cv2.rectangle(frame, (cx-obj_w//2, cy-obj_h//2), (cx+obj_w//2, cy+obj_h//2), (0,255,0), 3)
                    cv2.putText(frame, "MASTER", (cx, cy-20), 0, 1, (0,255,0), 2)
                    
                    err = cx - w//2
                    turn = err * Kp_turn
                    target_w = 200 
                    fwd = 0
                    if obj_w < target_w - 20: 
                        # [修改] 跟随速度加大
                        fwd = FOLLOW_SPEED 
                    elif obj_w > target_w + 20: 
                        # [修改] 后退保持速度加大
                        fwd = BACKUP_SPEED * 0.5 
                    
                    robot_move(fwd+turn, fwd-turn)
                else:
                    robot_stop()
                    cv2.putText(frame, "LOST MASTER", (10, 60), 0, 1, (0,0,255), 2)

            cv2.imshow("Turbo Robot", frame)
            if cv2.waitKey(1) == ord('q'): break
            
    except KeyboardInterrupt:
        pass
    finally:
        robot_stop()
        cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()