import asyncio
import serial
import time
import threading
import pygame
import os
from bleak import BleakScanner

# ================= 1. 配置区 (Config) =================
# --- 蓝牙与逻辑配置 ---
TARGET_NAME = "FollowMe_Target" # 必须和 Xiao 代码一致
OBSTACLE_DIST = 20.0            # 避障距离 (cm)
SAFE_DIST = 30.0                # 安全距离
TARGET_RSSI_STOP = -50          # 抓到了 (信号强)
TARGET_RSSI_LOST = -95          # 丢了 (信号弱)

# --- 串口设置 ---
SERIAL_PORT = '/dev/ttyACM0'    # 树莓派上的端口 (如果在电脑测试请改回 COMx)
BAUD_RATE = 115200

# --- 表情状态定义 ---
FACE_ANGRY = "angry"       # 追赶中
FACE_CONFUSED = "confused" # 迷茫/避障
FACE_HAPPY = "happy"       # 抓到了
# FACE_NORMAL = "normal"   # (已弃用，简化逻辑)

# ================= 2. 全局变量 (Shared Data) =================
current_distance = 999.0  # 超声波距离
latest_rssi = -100        # 蓝牙信号强度
current_face = FACE_CONFUSED # 默认先疑惑
is_running = True         # 系统运行开关

# ================= 3. 表情显示线程 (Display Thread) =================
def display_loop():
    """
    负责在屏幕上一直画图，不干扰主逻辑
    """
    pygame.init()
    # 树莓派上全屏，电脑上窗口
    try:
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    except:
        screen = pygame.display.set_mode((800, 480))
        
    pygame.mouse.set_visible(False) # 隐藏鼠标
    w, h = screen.get_size()

    # 加载图片 helper
    def load_img(name):
        path = f"assets/{name}.png"
        try:
            img = pygame.image.load(path)
            return pygame.transform.scale(img, (w, h))
        except:
            print(f"⚠️ 缺失图片: {path}")
            s = pygame.Surface((w, h))
            s.fill((100, 100, 100)) # 缺失显示灰色
            return s

    # 预加载表情
    faces = {
        FACE_ANGRY:    load_img("angry"),
        FACE_CONFUSED: load_img("confused"),
        FACE_HAPPY:    load_img("happy")
    }

    print(">>> 显示屏系统启动完成")

    while is_running:
        # 根据全局变量 current_face 刷新屏幕
        if current_face in faces:
            screen.blit(faces[current_face], (0, 0))
        
        # (可选) 显示调试文字，不用的话可以注释掉
        font = pygame.font.SysFont(None, 40)
        status_text = f"RSSI: {latest_rssi} | Dist: {current_distance:.1f}"
        text_surf = font.render(status_text, True, (0, 255, 0))
        screen.blit(text_surf, (10, 10))

        pygame.display.update()
        time.sleep(0.1) 

    pygame.quit()

# ================= 4. 传感器与通信逻辑 =================
def stm32_listener(ser):
    """ 监听 STM32 发来的距离数据 """
    global current_distance
    while is_running:
        try:
            if ser and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "Distance" in line:
                    parts = line.split(':')
                    if len(parts) > 1:
                        current_distance = float(parts[1].replace('cm', '').strip())
        except Exception as e:
            # print(f"Serial Read Error: {e}")
            time.sleep(1)

def send_command(ser, cmd):
    """ 发送指令给 STM32 """
    if ser:
        try:
            ser.write(cmd.encode())
        except:
            pass
    else:
        print(f"[Simulated Motor] >> {cmd}")

async def scan_for_target():
    """ 蓝牙扫描 """
    global latest_rssi
    print(">>> 开始扫描蓝牙信号...")
    
    def callback(device, advertisement_data):
        global latest_rssi
        if device.name == TARGET_NAME:
            latest_rssi = device.rssi

    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    
    while is_running:
        await asyncio.sleep(1)

# ================= 5. 主大脑 (Brain & Decision) =================
async def robot_brain():
    global current_face, is_running, current_distance
    
    # --- 连接硬件 ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f">>> STM32 连接成功: {SERIAL_PORT}")
    except:
        print(">>> ⚠️ 未找到 STM32，进入仿真模式 (Simulation Mode)")
        ser = None 

    # --- 启动辅助线程 ---
    if ser:
        t_serial = threading.Thread(target=stm32_listener, args=(ser,), daemon=True)
        t_serial.start()
    
    t_display = threading.Thread(target=display_loop, daemon=True)
    t_display.start()

    asyncio.create_task(scan_for_target())

    print(">>> 机器人全系统启动 (按 Ctrl+C 退出)")
    state = "INIT"

    # --- 主循环 ---
    while is_running:
        
        # 1. 避障模式 (危险！)
        if current_distance < OBSTACLE_DIST:
            if state != "AVOID":
                print(f"⚠️ 障碍物! 停车避让")
                state = "AVOID"
                current_face = FACE_CONFUSED # 也可以改成 "吓一跳" 的表情
                
                send_command(ser, 's') 
                await asyncio.sleep(0.2)
                send_command(ser, 'a') # 旋转避让
            
            await asyncio.sleep(0.1)

        # 2. 正常逻辑
        else:
            # A. 丢失目标 -> 疑惑
            if latest_rssi < TARGET_RSSI_LOST:
                if state != "SEARCH":
                    print(">>> 目标丢失，搜索中...")
                    state = "SEARCH"
                    current_face = FACE_CONFUSED 
                    send_command(ser, 'a') # 原地转圈找
            
            # B. 抓到了 -> 开心
            elif latest_rssi > TARGET_RSSI_STOP:
                if state != "STOP":
                    print(">>> 抓到了！(Happy)")
                    state = "STOP"
                    current_face = FACE_HAPPY 
                    send_command(ser, 's') 
            
            # C. 追逐中 -> 愤怒 (简化版逻辑)
            else:
                if state != "CHASE":
                    print(f">>> 发现目标 (RSSI {latest_rssi})，愤怒追赶！")
                    state = "CHASE"
                    current_face = FACE_ANGRY # <--- 关键修改：直接变愤怒
                    send_command(ser, 'w') 

        await asyncio.sleep(0.1) # 决策频率

# ================= 6. 入口 =================
if __name__ == "__main__":
    try:
        asyncio.run(robot_brain())
    except KeyboardInterrupt:
        print("\n>>> 程序停止")
        is_running = False