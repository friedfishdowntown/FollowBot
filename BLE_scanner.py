import asyncio
from bleak import BleakScanner
from config import *
"""
用于step 1确认target person在哪
"""

class BLETargetFinder:
    """蓝牙目标定位器"""
    
    def __init__(self):
        self.target_found = False
        self.best_rssi = -100
        self.scan_duration = RSSI_SCAN_TIME
        
    async def scan_for_target(self):
        """
        扫描蓝牙设备,寻找目标
        
        返回:
            found: bool
            rssi: int (信号强度)
        """
        print(f"\n Scanning for BLE device: {TARGET_BLE_NAME}")
        print(f"   Scan duration: {self.scan_duration}s")
        
        self.target_found = False
        self.best_rssi = -100
        
        def detection_callback(device, advertisement_data):
            if device.name == TARGET_BLE_NAME:
                self.target_found = True
                if device.rssi > self.best_rssi:
                    self.best_rssi = device.rssi
                    print(f"   Signal: {device.rssi} dBm")
        
        scanner = BleakScanner(detection_callback=detection_callback)
        
        await scanner.start()
        await asyncio.sleep(self.scan_duration)
        await scanner.stop()
        
        if self.target_found:
            print(f"✓ Target found! Best RSSI: {self.best_rssi} dBm")
            return True, self.best_rssi
        else:
            print("✗ Target not found")
            return False, -100
    
    async def find_strongest_direction(self, motion_controller, num_rotations=8):
        """
        旋转机器人,找到信号最强的方向
        在人圈内原地转
        
        args:
            motion_controller: MotionController实例
            num_rotations: 分成多少个方向扫描 = 多少个人
        
        return:
            best_direction: 0 到 num_rotations-1
        """
        print(f"\n Rotating to find strongest signal...")
        
        rotation_step = 360 / num_rotations  # 每次旋转角度
        rotation_time = rotation_step / 360 * 2.0  # 假设转一圈需要2秒
        
        best_direction = 0
        best_rssi = -100
        
        for i in range(num_rotations):
            print(f"  Direction {i+1}/{num_rotations}...", end='')
            
            # 扫描当前方向
            found, rssi = await self.scan_for_target()
            
            if rssi > best_rssi:
                best_rssi = rssi
                best_direction = i
                print(f" ✓ New best: {rssi} dBm")
            else:
                print(f" {rssi} dBm")
            
            # 旋转到下一个方向
            if i < num_rotations - 1:
                motion_controller.rotate_in_place('right')
                await asyncio.sleep(rotation_time)
                motion_controller.stop()
                await asyncio.sleep(0.5)
        
        print(f"\n✓ Best direction: {best_direction} (RSSI: {best_rssi} dBm)")
        
        # 旋转回到最佳方向
        steps_back = (num_rotations - best_direction) % num_rotations
        if steps_back > 0:
            print(f"  Rotating back {steps_back} steps...")
            motion_controller.rotate_in_place('right')
            await asyncio.sleep(rotation_time * steps_back)
            motion_controller.stop()
        
        return best_direction
    
    def run_sync_scan(self):
        """同步版本的扫描 (在主循环中使用)"""
        return asyncio.run(self.scan_for_target())