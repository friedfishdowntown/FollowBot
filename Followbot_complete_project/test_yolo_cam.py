import cv2
import threading
import time
import numpy as np
from ultralytics import YOLO

# ================= 配置区域 =================
# 替换为你手机 APP 里的 MJPEG 地址
IP_CAMERA_URL = "http://172.26.33.3/video.mjpg" 

# 跳帧设置：每 X 帧侦测一次
# 值越大越流畅，但框的跟随延迟越高
SKIP_FRAMES = 3 

# YOLO 推理尺寸 (320 是速度与精度的平衡点，640 会慢很多)
INFERENCE_SIZE = 320

# ================= 1. 防延迟视频流类 =================
class IPCameraStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        if not self.stream.isOpened():
            print("Error: Could not open video stream.")
            self.frame = None
            self.grabbed = False
        else:
            (self.grabbed, self.frame) = self.stream.read()
            
        self.stopped = False
        self.t = threading.Thread(target=self.update, daemon=True)
        self.t.start()

    def update(self):
        while not self.stopped:
            if not self.stream.isOpened(): continue
            # 只要有新帧就立刻读取，覆盖旧帧
            (grabbed, frame) = self.stream.read()
            if grabbed:
                self.grabbed = grabbed
                self.frame = frame
            else:
                time.sleep(0.01)

    def read(self):
        return self.grabbed, self.frame

    def release(self):
        self.stopped = True
        self.t.join()
        self.stream.release()

# ================= 2. 主程序 =================
def main():
    print("Loading YOLOv8n model...")
    # 加载模型
    model = YOLO("yolov8n.pt") 
    
    print(f"Connecting to {IP_CAMERA_URL}...")
    cam_stream = IPCameraStream(IP_CAMERA_URL)
    
    # 等待摄像头连接稳定
    time.sleep(1.0)
    
    frame_count = 0
    last_boxes = [] # 用于缓存上一次检测到的框 [x1, y1, x2, y2]
    
    # FPS 计算变量
    prev_time = 0
    curr_fps = 0

    print(f"Running... Detecting every {SKIP_FRAMES} frames.")
    print("Press 'q' to exit.")

    try:
        while True:
            # 1. 获取最新画面
            ret, frame = cam_stream.read()
            if not ret or frame is None:
                continue

            display_frame = frame.copy()

            # 2. 跳帧逻辑
            if frame_count % SKIP_FRAMES == 0:
                # === 执行 YOLO 推理 (耗时操作) ===
                # imgsz=320: 牺牲远距离微小物体检测率，换取极大速度提升
                # classes=0: 只检测人
                results = model.predict(frame, classes=0, imgsz=INFERENCE_SIZE, verbose=False, conf=0.5)
                
                # 更新缓存的框
                current_boxes = []
                if len(results) > 0 and results[0].boxes is not None:
                    for box in results[0].boxes:
                        # 获取坐标 (x1, y1, x2, y2)
                        coords = box.xyxy[0].cpu().numpy().astype(int)
                        conf = float(box.conf[0])
                        current_boxes.append((coords, conf))
                
                last_boxes = current_boxes
            
            # 3. 绘制逻辑 (每一帧都执行)
            # 无论这一帧有没有跑 YOLO，都把 last_boxes 画上去
            # 这样画面看起来就是连续有框的，不会闪烁
            for (coords, conf) in last_boxes:
                x1, y1, x2, y2 = coords
                # 画矩形
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 画标签
                label = f"Person {conf:.2f}"
                cv2.putText(display_frame, label, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 4. 计算并显示 FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time
            cv2.putText(display_frame, f"FPS: {fps:.1f} (Skip:{SKIP_FRAMES})", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 5. 显示
            cv2.imshow("YOLO Optimization Test", display_frame)
            
            frame_count += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cam_stream.release()
        cv2.destroyAllWindows()
        print("Exited.")

if __name__ == "__main__":
    main()