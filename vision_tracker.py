import cv2
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2
from config import *
import time

class VisionTracker:
    """视觉追踪器 - 负责目标检测、识别和追踪"""
    
    def __init__(self):
        # 初始化摄像头
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()
        time.sleep(2)  # 等待摄像头预热
        print("✓ Camera initialized")
        
        # 初始化YOLO
        self.yolo = YOLO(YOLO_MODEL)
        print(f"✓ YOLO model loaded: {YOLO_MODEL}")
        
        # 目标特征存储
        self.target_upper_color = None  # 上身颜色直方图
        self.target_lower_color = None  # 下身颜色直方图
        self.target_captured = False
        
        # 追踪状态
        self.lost_frame_count = 0
        
    def capture_frame(self):
        """捕获一帧图像"""
        frame = self.camera.capture_array()
        return frame
    
    def detect_people(self, frame):
        """
        使用YOLO检测画面中的所有人
        
        返回:
            detections: list of dict, 每个包含 {bbox, confidence}
                bbox = (x, y, w, h)
        """
        results = self.yolo(frame, conf=YOLO_CONFIDENCE, verbose=False)
        
        people = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # 只保留"人"类别 (class_id = 0)
                if int(box.cls[0]) == 0:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    w = x2 - x1
                    h = y2 - y1
                    conf = float(box.conf[0])
                    
                    people.append({
                        'bbox': (int(x1), int(y1), int(w), int(h)),
                        'confidence': conf
                    })
        
        return people
    
    def extract_clothing_color(self, frame, bbox):
        """
        提取人物的上下身颜色特征
        
        参数:
            frame: 图像
            bbox: (x, y, w, h)
        
        返回:
            upper_hist: 上身颜色直方图
            lower_hist: 下身颜色直方图
        """
        x, y, w, h = bbox
        
        # 提取人物区域
        person_roi = frame[y:y+h, x:x+w]
        
        # 分割上下身
        upper_h = int(h * UPPER_BODY_RATIO)
        upper_roi = person_roi[:upper_h, :]
        lower_roi = person_roi[upper_h:, :]
        
        # 转换到HSV空间 (对光照变化更鲁棒)
        upper_hsv = cv2.cvtColor(upper_roi, cv2.COLOR_RGB2HSV)
        lower_hsv = cv2.cvtColor(lower_roi, cv2.COLOR_RGB2HSV)
        
        # 计算颜色直方图 (H通道)
        upper_hist = cv2.calcHist([upper_hsv], [0], None, [32], [0, 180])
        lower_hist = cv2.calcHist([lower_hsv], [0], None, [32], [0, 180])
        
        # 归一化
        cv2.normalize(upper_hist, upper_hist, 0, 1, cv2.NORM_MINMAX)
        cv2.normalize(lower_hist, lower_hist, 0, 1, cv2.NORM_MINMAX)
        
        return upper_hist, lower_hist
    
    def compare_clothing(self, upper_hist, lower_hist):
        """
        比较颜色直方图相似度
        
        返回:
            similarity: 0-1, 越高越相似
        """
        if self.target_upper_color is None:
            return 0.0
        
        # 使用相关性比较直方图
        upper_sim = cv2.compareHist(
            upper_hist, 
            self.target_upper_color, 
            cv2.HISTCMP_CORREL
        )
        lower_sim = cv2.compareHist(
            lower_hist, 
            self.target_lower_color, 
            cv2.HISTCMP_CORREL
        )
        
        # 加权平均 (上身权重更高,因为更稳定)
        similarity = 0.6 * upper_sim + 0.4 * lower_sim
        
        return similarity
    
    def capture_target_features(self):
        """
        初始化阶段: 拍摄多张照片,记录目标特征
        
        返回:
            success: bool
        """
        print(f"\n📸 Capturing target features ({NUM_INIT_SAMPLES} samples)...")
        
        upper_hists = []
        lower_hists = []
        
        for i in range(NUM_INIT_SAMPLES):
            print(f"  Sample {i+1}/{NUM_INIT_SAMPLES}...", end='')
            
            # 拍摄一帧
            frame = self.capture_frame()
            
            # 检测人
            people = self.detect_people(frame)
            
            if len(people) == 0:
                print(" ✗ No person detected!")
                time.sleep(1)
                continue
            
            # 选择最大的人框 (假设是目标)
            largest = max(people, key=lambda p: p['bbox'][2] * p['bbox'][3])
            
            # 提取颜色特征
            upper_hist, lower_hist = self.extract_clothing_color(
                frame, largest['bbox']
            )
            
            upper_hists.append(upper_hist)
            lower_hists.append(lower_hist)
            
            print(" ✓")
            time.sleep(0.5)
        
        if len(upper_hists) < 2:
            print("✗ Failed to capture enough samples!")
            return False
        
        # 取中位数作为目标特征 (去除异常值)
        self.target_upper_color = np.median(upper_hists, axis=0)
        self.target_lower_color = np.median(lower_hists, axis=0)
        self.target_captured = True
        
        print("✓ Target features captured!")
        return True
    
    def find_target(self, frame):
        """
        在画面中寻找目标人物
        
        返回:
            target: dict or None
                {
                    'bbox': (x, y, w, h),
                    'center': (cx, cy),
                    'area': int,
                    'similarity': float
                }
        """
        if not self.target_captured:
            return None
        
        # 检测所有人
        people = self.detect_people(frame)
        
        if len(people) == 0:
            self.lost_frame_count += 1
            return None
        
        # 与目标匹配
        best_match = None
        best_similarity = 0.0
        
        for person in people:
            bbox = person['bbox']
            
            # 提取颜色特征
            upper_hist, lower_hist = self.extract_clothing_color(frame, bbox)
            
            # 计算相似度
            similarity = self.compare_clothing(upper_hist, lower_hist)
            
            if similarity > best_similarity:
                best_similarity = similarity
                best_match = person
        
        # 检查是否足够相似
        if best_similarity < COLOR_MATCH_THRESHOLD:
            self.lost_frame_count += 1
            return None
        
        # 找到目标!
        self.lost_frame_count = 0
        
        bbox = best_match['bbox']
        x, y, w, h = bbox
        
        return {
            'bbox': bbox,
            'center': (x + w//2, y + h//2),
            'area': w * h,
            'similarity': best_similarity
        }
    
    def estimate_distance_from_bbox(self, bbox_area):
        """
        根据人框面积估算距离
        
        这需要预先标定! 这里用简单的反比例关系
        
        参数:
            bbox_area: 人框面积 (像素²)
        
        返回:
            distance: 估算距离 (cm)
        """
        # 简化模型: distance ∝ 1/√area
        # 假设在100cm时,bbox面积约为15000像素²
        REFERENCE_AREA = 15000
        REFERENCE_DISTANCE = 100
        
        if bbox_area < 1000:  # 防止除零
            return 300
        
        estimated_dist = REFERENCE_DISTANCE * np.sqrt(REFERENCE_AREA / bbox_area)
        
        return estimated_dist
    
    def draw_debug_info(self, frame, target, offset_x, steering):
        """
        在图像上绘制调试信息
        
        参数:
            frame: 图像
            target: 目标信息 (或None)
            offset_x: 水平偏移
            steering: 转向量
        """
        debug_frame = frame.copy()
        
        # 画中心线
        cv2.line(debug_frame, 
                 (CAMERA_WIDTH//2, 0), 
                 (CAMERA_WIDTH//2, CAMERA_HEIGHT),
                 (0, 255, 0), 2)
        
        # 画死区
        deadzone_left = CAMERA_WIDTH//2 - CENTER_DEADZONE
        deadzone_right = CAMERA_WIDTH//2 + CENTER_DEADZONE
        cv2.rectangle(debug_frame,
                      (deadzone_left, 0),
                      (deadzone_right, CAMERA_HEIGHT),
                      (0, 255, 0), 1)
        
        if target:
            # 画人框
            x, y, w, h = target['bbox']
            cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
            
            # 画中心点
            cx, cy = target['center']
            cv2.circle(debug_frame, (cx, cy), 5, (255, 0, 0), -1)
            
            # 画连线
            cv2.line(debug_frame,
                     (CAMERA_WIDTH//2, CAMERA_HEIGHT//2),
                     (cx, cy),
                     (255, 0, 0), 2)
            
            # 显示信息
            info_text = f"Sim: {target['similarity']:.2f} | Area: {target['area']}"
            cv2.putText(debug_frame, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            offset_text = f"Offset: {offset_x:.0f}px | Steer: {steering:.2f}"
            cv2.putText(debug_frame, offset_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(debug_frame, "TARGET LOST", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return debug_frame
    
    def cleanup(self):
        """清理资源"""
        self.camera.stop()
        print("Camera stopped")