from picamera2 import Picamera2
import time
def test_camera():
    """接收并显示摄像头画面"""
    
    print("=== Camera 测试 ===")
    picam2 = Picamera2()
    
    # 配置预览
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    
    # 开始预览
    picam2.start()
    print("摄像头预览已启动，将运行10秒...")
    time.sleep(10)
    
    # 拍摄一张照片保存
    picam2.capture_file("test_image.jpg")
    print("已保存测试图片: test_image.jpg")
    
    picam2.stop()

if __name__ == "__main__":
    try:
        test_camera()
    except Exception as e:
        print(f"错误: {e}")