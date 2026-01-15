#!/usr/bin/env python3
"""
人物追踪机器人 - 主控制程序
结合蓝牙定位、视觉识别、距离传感器的智能追踪系统
"""

import asyncio
import time
import cv2
import numpy as np
from config import *
from STM32_Motor_Controller import STM32Controller, MotionController
from vision_tracker import VisionTracker
from BLE_scanner import BLETargetFinder

class RobotTracker:
    """主控制器 - 状态机和决策逻辑"""
    
    def __init__(self):
        # 初始化各个模块
        print("=" * 50)
        print("🤖 Robot Tracker Initialization")
        print("=" * 50)
        
        self.stm32 = STM32Controller()
        self.motion = MotionController(self.stm32)
        self.vision = VisionTracker()
        self.ble = BLETargetFinder()
        
        # 状态机
        self.state = STATE_INIT
        self.previous_state = None
        
        # 控制变量
        self.running = True
        
    async def initialize(self):
        """初始化流程"""
        print("\n[INIT] Starting initialization...")
        
        # 1. 连接STM32
        if not self.stm32.connect():
            print("✗ Failed to connect STM32. Abort.")
            return False
        
        # 2. 测试电机
        print("\n[INIT] Testing motors...")
        self.motion.stop()
        time.sleep(0.5)
        
        print("✓ Initialization complete!\n")
        return True
    
    async def state_ble_search(self):
        """状态: 蓝牙搜索目标"""
        print("\n" + "="*50)
        print("[BLE_SEARCH] Searching for target...")
        print("="*50)
        
        # 扫描蓝牙
        found, rssi = await self.ble.scan_for_target()
        
        if not found or rssi < RSSI_THRESHOLD_FOUND:
            print("✗ Target not found or signal too weak. Retrying...")
            await asyncio.sleep(2)
            return STATE_BLE_SEARCH
        
        # 找到信号,开始旋转找最强方向
        await self.ble.find_strongest_direction(self.motion, num_rotations=8)
        
        return STATE_ALIGN
    
    async def state_align(self):
        """状态: 对准目标"""
        print("\n" + "="*50)
        print("[ALIGN] Aligning to target...")
        print("="*50)
        
        # 已经通过蓝牙旋转到最强方向了
        # 现在需要微调,让目标进入画面中心
        
        align_timeout = 10  # 10秒对准超时
        start_time = time.time()
        
        while time.time() - start_time < align_timeout:
            frame = self.vision.capture_frame()
            people = self.vision.detect_people(frame)
            
            if len(people) == 0:
                print("  No person in view, rotating slowly...")
                self.motion.rotate_in_place('right')
                await asyncio.sleep(0.3)
                self.motion.stop()
                continue
            
            # 找到最大的人框
            largest = max(people, key=lambda p: p['bbox'][2] * p['bbox'][3])
            x, y, w, h = largest['bbox']
            cx = x + w // 2
            
            offset_x = cx - CAMERA_WIDTH // 2
            
            if abs(offset_x) < CENTER_DEADZONE:
                print("✓ Target centered!")
                self.motion.stop()
                return STATE_CAPTURE
            
            # 微调方向
            if offset_x > 0:
                print(f"  Adjusting right (offset: {offset_x}px)...")
                self.motion.rotate_in_place('right')
            else:
                print(f"  Adjusting left (offset: {offset_x}px)...")
                self.motion.rotate_in_place('left')
            
            await asyncio.sleep(0.1)
            self.motion.stop()
            await asyncio.sleep(0.2)
        
        print("✗ Alignment timeout. Retrying BLE search...")
        return STATE_BLE_SEARCH
    
    async def state_capture(self):
        """状态: 拍摄记录目标特征"""
        print("\n" + "="*50)
        print("[CAPTURE] Capturing target features...")
        print("="*50)
        print("⚠️  Please ensure target person is in front of camera!")
        await asyncio.sleep(2)
        
        success = self.vision.capture_target_features()
        
        if not success:
            print("✗ Failed to capture target. Retrying alignment...")
            return STATE_ALIGN
        
        print("\n✓ Ready to track!")
        await asyncio.sleep(1)
        return STATE_TRACK
    
    async def state_track(self):
        """状态: 视觉追踪"""
        if self.previous_state != STATE_TRACK:
            print("\n" + "="*50)
            print("[TRACK] Starting visual tracking...")
            print("="*50)
        
        frame = self.vision.capture_frame()
        target = self.vision.find_target(frame)
        
        # ========== 目标丢失处理 ==========
        if target is None:
            if self.vision.lost_frame_count > LOST_FRAME_THRESHOLD:
                print("\n⚠️  Target lost for too long!")
                return STATE_LOST
            else:
                # 短暂丢失,保持最后方向慢速前进
                print(f"⚠️  Target lost ({self.vision.lost_frame_count}/{LOST_FRAME_THRESHOLD})")
                self.motion.move_with_steering(MIN_SPEED * 0.5, 0)
                return STATE_TRACK
        
        # ========== 找到目标,开始追踪 ==========
        cx, cy = target['center']
        bbox_area = target['area']
        
        # 1. 计算画面偏移
        offset_x = cx - CAMERA_WIDTH // 2
        offset_y = cy - CAMERA_HEIGHT // 2
        
        # 2. 计算转向量 (PID控制)
        steering = self.motion.calculate_steering(offset_x)
        
        # 3. 距离融合估算
        visual_distance = self.vision.estimate_distance_from_bbox(bbox_area)
        sonar_distance = self.stm32.get_distance()
        
        # 如果基本对准,融合超声波数据
        if abs(offset_x) < CENTER_DEADZONE * 2:
            # 目标在正前方,超声波可信
            confidence_sonar = 0.7
            confidence_visual = 0.3
        else:
            # 目标偏离,主要靠视觉
            confidence_sonar = 0.2
            confidence_visual = 0.8
        
        fused_distance = (confidence_sonar * sonar_distance + 
                          confidence_visual * visual_distance)
        
        # 4. 根据距离决定速度
        if fused_distance < SAFE_DISTANCE:
            # 太近,停止
            speed = 0
            print(f"🛑 STOP - Too close ({fused_distance:.1f}cm)")
            self.motion.stop()
            
        elif fused_distance < FOLLOW_DISTANCE:
            # 理想距离,慢速跟随
            speed = MIN_SPEED
            print(f"🚶 FOLLOW - Distance: {fused_distance:.1f}cm | Steer: {steering:.2f}")
            self.motion.move_with_steering(speed, steering)
            
        elif fused_distance < MAX_DISTANCE:
            # 稍远,中速追赶
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.5
            print(f"🏃 CHASE - Distance: {fused_distance:.1f}cm | Steer: {steering:.2f}")
            self.motion.move_with_steering(speed, steering)
            
        else:
            # 很远,全速追赶
            speed = MAX_SPEED
            print(f"🚀 SPRINT - Distance: {fused_distance:.1f}cm | Steer: {steering:.2f}")
            self.motion.move_with_steering(speed, steering)
        
        # 5. 调试显示
        if DEBUG_MODE and SAVE_DEBUG_IMAGES:
            debug_frame = self.vision.draw_debug_info(
                frame, target, offset_x, steering
            )
            cv2.imshow("Debug", debug_frame)
            cv2.waitKey(1)
        
        return STATE_TRACK
    
    async def state_lost(self):
        """状态: 目标丢失,搜索模式"""
        print("\n" + "="*50)
        print("[LOST] Target lost. Searching...")
        print("="*50)
        
        search_timeout = 15  # 15秒搜索超时
        start_time = time.time()
        
        while time.time() - start_time < search_timeout:
            # 原地旋转搜索
            self.motion.rotate_in_place('right')
            await asyncio.sleep(0.5)
            self.motion.stop()
            
            # 检查是否重新找到
            frame = self.vision.capture_frame()
            target = self.vision.find_target(frame)
            
            if target is not None:
                print("✓ Target re-acquired!")
                self.vision.lost_frame_count = 0
                self.motion.reset_pid()
                return STATE_TRACK
            
            await asyncio.sleep(0.5)
        
        print("✗ Search timeout. Returning to BLE search...")
        return STATE_BLE_SEARCH
    
    async def run(self):
        """主循环"""
        # 初始化
        if not await self.initialize():
            return
        
        # 状态机循环
        self.state = STATE_BLE_SEARCH
        
        try:
            while self.running:
                self.previous_state = self.state
                
                # 状态分发
                if self.state == STATE_BLE_SEARCH:
                    self.state = await self.state_ble_search()
                
                elif self.state == STATE_ALIGN:
                    self.state = await self.state_align()
                
                elif self.state == STATE_CAPTURE:
                    self.state = await self.state_capture()
                
                elif self.state == STATE_TRACK:
                    self.state = await self.state_track()
                    await asyncio.sleep(CONTROL_DT)  # 控制循环频率
                
                elif self.state == STATE_LOST:
                    self.state = await self.state_lost()
                
                elif self.state == STATE_STOP:
                    print("\n[STOP] Robot stopped.")
                    self.motion.stop()
                    break
                
        except KeyboardInterrupt:
            print("\n\n  Keyboard interrupt detected!")
        
        finally:
            # 清理
            print("\n" + "="*50)
            print("Shutting down...")
            print("="*50)
            self.motion.stop()
            self.stm32.disconnect()
            self.vision.cleanup()
            if SAVE_DEBUG_IMAGES:
                cv2.destroyAllWindows()
            print("✓ Shutdown complete.")


# ========== 程序入口 ==========
if __name__ == "__main__":
    print("""
    ╔════════════════════════════════════════╗
    ║   Visual Tracking Robot System         ║
    ║   Press Ctrl+C to stop                 ║
    ╚════════════════════════════════════════╝
    """)
    
    tracker = RobotTracker()
    asyncio.run(tracker.run())