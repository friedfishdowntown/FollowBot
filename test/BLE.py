from bluepy.btle import Scanner, DefaultDelegate

def test_ble_beacon():
    """扫描并读取BLE Beacon信号强度"""
    
    class ScanDelegate(DefaultDelegate):
        def __init__(self):
            DefaultDelegate.__init__(self)
    
    print("=== BLE Beacon 测试 ===")
    scanner = Scanner().withDelegate(ScanDelegate())
    devices = scanner.scan(5.0)  # 扫描5秒
    
    for dev in devices:
        print(f"地址: {dev.addr}, RSSI: {dev.rssi} dB")
        for (adtype, desc, value) in dev.getScanData():
            print(f"  {desc}: {value}")

if __name__ == "__main__":
    try:
        test_ble_beacon()
    except Exception as e:
        print(f"错误: {e}")