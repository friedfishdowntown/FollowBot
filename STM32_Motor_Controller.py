import serial
import threading
import time
from config import *

class STM32Controller:
    """STM32通信控制器 - 负责串口通信和距离数据接收"""
    
    def __init__(self):
        self.ser = None
        self.current_distance = 999.0  # 当前超声波距离 (cm)
        self.listener_thread = None
        self.running = False
        
    def connect(self):
        """连接到STM32"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # 等待串口稳定
            print(f"✓ Connected to STM32 on {SERIAL_PORT}")
            
            # 启动监听线程
            self.running = True
            self.listener_thread = threading.Thread(
                target=self._listen_loop, 
                daemon=True
            )
            self.listener_thread.start()
            return True
            
        except Exception as e:
            print(f"✗ STM32 Connection Failed: {e}")
            return False
    
    def _listen_loop(self):
        """监听线程 - 持续接收STM32的距离数据"""
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    # 读取一行数据
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 解析格式: "Distance: 25.4 cm"
                    if "Distance" in line:
                        parts = line.split(':')
                        if len(parts) > 1:
                            dist_str = parts[1].replace('cm', '').strip()
                            self.current_distance = float(dist_str)
                            if DEBUG_MODE:
                                print(f"📏 Distance: {self.current_distance:.1f} cm")
                                
            except Exception as e:
                if DEBUG_MODE:
                    print(f"Serial Read Error: {e}")
                time.sleep(0.1)
    
    def send_motor_command(self, left_pwm, right_pwm):
        """
        发送电机控制指令
        
        参数:
            left_pwm: 左轮速度 (-100 到 100, 负数为后退)
            right_pwm: 右轮速度 (-100 到 100)
        """
        if not self.ser:
            return False
        
        try:
            # 限制范围
            left_pwm = max(-100, min(100, int(left_pwm)))
            right_pwm = max(-100, min(100, int(right_pwm)))
            
            # 构造指令: "M:L80,R50"
            cmd = f"M:L{left_pwm},R{right_pwm}\n"
            self.ser.write(cmd.encode())
            
            if DEBUG_MODE:
                print(f"🤖 Motor: L={left_pwm}, R={right_pwm}")
            return True
            
        except Exception as e:
            print(f"Motor Command Error: {e}")
            return False
    
    def stop(self):
        """紧急停止"""
        return self.send_motor_command(0, 0)
    
    def get_distance(self):
        """获取当前距离"""
        return self.current_distance
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.listener_thread:
            self.listener_thread.join(timeout=1)
        if self.ser:
            self.stop()
            self.ser.close()
            print("STM32 Disconnected")


class MotionController:
    """运动控制器 - 高级运动控制逻辑"""
    
    def __init__(self, stm32: STM32Controller):
        self.stm32 = stm32
        
        # PID 控制器状态
        self.last_error = 0
        self.integral = 0
        
    def reset_pid(self):
        """重置PID控制器"""
        self.last_error = 0
        self.integral = 0
    
    def calculate_steering(self, offset_x):
        """
        PID控制计算转向
        
        参数:
            offset_x: 目标相对画面中心的水平偏移 (像素)
        
        返回:
            steering: -1.0 到 1.0 (-1=全速左转, 1=全速右转)
        """
        # PID 计算
        error = offset_x
        self.integral += error * CONTROL_DT
        derivative = (error - self.last_error) / CONTROL_DT
        
        # PID输出
        output = (PID_KP * error + 
                  PID_KI * self.integral + 
                  PID_KD * derivative)
        
        self.last_error = error
        
        # 归一化到 -1.0 ~ 1.0
        steering = max(-1.0, min(1.0, output / (CAMERA_WIDTH / 2)))
        
        return steering
    
    def move_with_steering(self, base_speed, steering):
        """
        差速转向运动
        
        参数:
            base_speed: 基础速度 (0-100)
            steering: 转向量 (-1.0 到 1.0)
        """
        if abs(steering) < 0.05:  # 死区
            # 直行
            self.stm32.send_motor_command(base_speed, base_speed)
        else:
            # 差速转向
            if steering > 0:  # 右转
                left_speed = base_speed
                right_speed = base_speed * (1 - abs(steering))
            else:  # 左转
                left_speed = base_speed * (1 - abs(steering))
                right_speed = base_speed
            
            self.stm32.send_motor_command(
                int(left_speed), 
                int(right_speed)
            )
    
    def rotate_in_place(self, direction):
        """
        原地旋转
        
        参数:
            direction: 'left' 或 'right'
        """
        if direction == 'left':
            self.stm32.send_motor_command(-SEARCH_ROTATION_SPEED, SEARCH_ROTATION_SPEED)
        else:
            self.stm32.send_motor_command(SEARCH_ROTATION_SPEED, -SEARCH_ROTATION_SPEED)
    
    def stop(self):
        """停止"""
        self.stm32.stop()
        self.reset_pid()