from PIL import Image, ImageDraw, ImageFont
import os

def test_display():
    """在HDMI显示屏上显示图像"""

    
    print("=== HDMI 显示屏测试 ===")
    
    # 创建一个测试图像 (800x480适合5寸屏)
    width, height = 800, 480
    img = Image.new('RGB', (width, height), color='blue')
    draw = ImageDraw.Draw(img)
    
    # 绘制一些图形
    draw.rectangle([50, 50, 750, 430], outline='yellow', width=5)
    draw.ellipse([200, 140, 600, 340], fill='red')
    
    # 添加文字
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 50)
    except:
        font = ImageFont.load_default()
    
    draw.text((250, 200), "Raspberry Pi 5", fill='white', font=font)
    
    # 保存图像
    img.save("display_test.png")
    print("测试图像已保存: display_test.png")
    
    # 使用feh或其他工具在HDMI上全屏显示
    # 需要安装: sudo apt-get install feh
    print("使用 feh 显示图像...")
    os.system("DISPLAY=:0 feh --fullscreen --auto-zoom display_test.png &")
    print("图像应该显示在HDMI屏幕上")

if __name__ == "__main__":
    try:
        test_display()
    except Exception as e:
        print(f"错误: {e}")