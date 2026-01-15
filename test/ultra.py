import RPi.GPIO as GPIO
import time

def test_ultrasonic():
    """读取超声波距离数据"""
    
    # GPIO引脚定义 (根据实际接线修改)
    TRIG = 23
    ECHO = 24
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    
    print("=== 超声波传感器测试 ===")
    
    def get_distance():
        # 发送触发信号
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)
        
        # 等待回波
        while GPIO.input(ECHO) == GPIO.LOW:
            pulse_start = time.time()
        
        while GPIO.input(ECHO) == GPIO.HIGH:
            pulse_end = time.time()
        
        # 计算距离 (声速 34300 cm/s)
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        
        return distance
    
    # 连续读取10次
    for i in range(10):
        dist = get_distance()
        print(f"测量 {i+1}: 距离 = {dist} cm")
        time.sleep(0.5)
    
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        test_ultrasonic()
    except Exception as e:
        print(f"错误: {e}")