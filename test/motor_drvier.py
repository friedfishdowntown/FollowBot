import RPi.GPIO as GPIO
import time

def test_motor_driver():
    """控制左右轮以不同速度转动"""
    
    # GPIO引脚定义 (根据实际接线修改)
    # 左电机
    LEFT_PWM = 12
    LEFT_IN1 = 17
    LEFT_IN2 = 27
    # 右电机
    RIGHT_PWM = 13
    RIGHT_IN1 = 22
    RIGHT_IN2 = 23
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([LEFT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2], GPIO.OUT)
    
    # 创建PWM对象
    left_pwm = GPIO.PWM(LEFT_PWM, 1000)  # 1kHz频率
    right_pwm = GPIO.PWM(RIGHT_PWM, 1000)
    left_pwm.start(0)
    right_pwm.start(0)
    
    print("=== 电机测试 ===")
    
    # 左轮速度60%, 右轮速度80%
    left_speed = 60
    right_speed = 80
    
    # 设置正转
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    
    # 设置速度
    left_pwm.ChangeDutyCycle(left_speed)
    right_pwm.ChangeDutyCycle(right_speed)
    
    print(f"左轮速度: {left_speed}%, 右轮速度: {right_speed}%")
    print("运行5秒...")
    time.sleep(5)
    
    # 停止
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
    print("电机已停止")

if __name__ == "__main__":
    try:
        test_motor_driver()
    except Exception as e:
        print(f"错误: {e}")