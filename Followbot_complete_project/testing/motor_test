import RPi.GPIO as GPIO
import time

def test_turning():
    """测试小车原地左转和右转功能"""
    
    # === 1. 引脚定义 (保持与您的一致) ===
    # 左电机 (Motor A)
    LEFT_PWM = 12
    LEFT_IN1 = 5
    LEFT_IN2 = 6
    
    # 右电机 (Motor B)
    RIGHT_PWM = 13
    RIGHT_IN1 = 22
    RIGHT_IN2 = 27
    
    # === 2. 初始化设置 ===
    GPIO.setmode(GPIO.BCM)
    pins = [LEFT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2]
    GPIO.setup(pins, GPIO.OUT)
    
    # 创建PWM对象 (1000Hz)
    left_pwm = GPIO.PWM(LEFT_PWM, 1000)
    right_pwm = GPIO.PWM(RIGHT_PWM, 1000)
    
    # 启动PWM，初始占空比0
    left_pwm.start(0)
    right_pwm.start(0)
    
    # 设置转向速度 (建议比直线速度稍大，因为摩擦力较大)
    TURN_SPEED = 60 
    
    print("=== 转向测试开始 ===")
    time.sleep(1)
    
    try:
        # =========================
        # 测试 1: 原地左转 (Spin Left)
        # 逻辑: 左轮后退，右轮前进
        # =========================
        print(f"1. 原地向左旋转 (速度 {TURN_SPEED}%)")
        
        # 左轮 -> 向后
        GPIO.output(LEFT_IN1, GPIO.LOW)
        GPIO.output(LEFT_IN2, GPIO.HIGH)
        left_pwm.ChangeDutyCycle(TURN_SPEED)
        
        # 右轮 -> 向前
        GPIO.output(RIGHT_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_IN2, GPIO.LOW)
        right_pwm.ChangeDutyCycle(TURN_SPEED)
        
        time.sleep(1.5) # 旋转1.5秒
        
        # --- 停止 ---
        print("   停止")
        left_pwm.ChangeDutyCycle(0)
        right_pwm.ChangeDutyCycle(0)
        time.sleep(1)

        # =========================
        # 测试 2: 原地右转 (Spin Right)
        # 逻辑: 左轮前进，右轮后退
        # =========================
        print(f"2. 原地向右旋转 (速度 {TURN_SPEED}%)")
        
        # 左轮 -> 向前
        GPIO.output(LEFT_IN1, GPIO.HIGH)
        GPIO.output(LEFT_IN2, GPIO.LOW)
        left_pwm.ChangeDutyCycle(TURN_SPEED)
        
        # 右轮 -> 向后
        GPIO.output(RIGHT_IN1, GPIO.LOW)
        GPIO.output(RIGHT_IN2, GPIO.HIGH)
        right_pwm.ChangeDutyCycle(TURN_SPEED)
        
        time.sleep(1.5)
        
        # --- 停止 ---
        print("   停止")
        left_pwm.ChangeDutyCycle(0)
        right_pwm.ChangeDutyCycle(0)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n用户强制中断")
        
    finally:
        # === 4. 停止与清理 (严格遵循您的修复方案) ===
        print("正在停止...")
        
        # 1. 停止 PWM
        left_pwm.stop()
        right_pwm.stop()
        
        # 2. 关键: 删除 PWM 对象
        try:
            del left_pwm
            del right_pwm
        except NameError:
            pass # 防止对象未创建时报错
            
        # 3. 清理 GPIO
        GPIO.cleanup()
        print("测试结束，已安全退出")

if __name__ == "__main__":
    test_turning()