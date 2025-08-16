#!/usr/bin/env python3
"""
改进的红色限高杆检测器
针对实际比赛环境优化：
- 限高杆尺寸：110cm高，10cm×10cm截面
- 颜色：红色但受室内光照影响呈现粉红/暗红色
- 底部距地面40cm
"""

import cv2
import numpy as np
import os
import sys

class ImprovedRedBarrierTester:
    def __init__(self):
        # 扩展的红色检测参数 - 更精确的红色范围
        # 主要红色范围（基于颜色分析结果 H=0-3）
        self.lower_red1 = np.array([0, 50, 50])      # 提高饱和度阈值排除黄色
        self.upper_red1 = np.array([10, 255, 255])   # 主要红色范围
        self.lower_red2 = np.array([170, 50, 50])    # 深红色范围
        self.upper_red2 = np.array([180, 255, 255])  
        
        # 橙红色范围（室内LED灯光下的红色）- 更严格
        self.lower_orange_red = np.array([0, 80, 80])    # 提高饱和度和亮度
        self.upper_orange_red = np.array([8, 255, 255])
        
        # 深红色范围（阴影中的红色）
        self.lower_dark_red = np.array([160, 40, 40])    # 稍微提高阈值
        self.upper_dark_red = np.array([180, 255, 200])
        
        # 调整检测参数适应实际限高杆
        self.min_contour_area = 100      # 适中最小面积，避免误检小噪点
        self.min_aspect_ratio = 0.05     # 稍微提高，排除过窄的线条
        self.max_aspect_ratio = 20.0     # 降低上限
        
        # 多层ROI策略 - 包括极近距离
        self.roi_configs = {
            'very_near': {'top': 0.0, 'bottom': 1.0},    # 极近距离：全图检测
            'near': {'top': 0.0, 'bottom': 0.8},         # 近距离：大部分区域
            'medium': {'top': 0.1, 'bottom': 0.7},       # 中距离：稍微上移
            'far': {'top': 0.2, 'bottom': 0.8},          # 远距离：中部区域
        } 
        
        # 距离估算参数
        self.reference_height_pixels = 60  # 调整参考高度（实际观测）
        self.reference_distance = 2.0      # 2米距离时的参考
        self.detection_threshold = 0.2     # 提高阈值，减少误检
        
        # 新增：形状验证参数 - 稍微严格一些
        self.min_width = 3               # 最小宽度
        self.max_width = 800             # 最大宽度
        self.min_height = 3              # 最小高度
        self.max_height = 800            # 最大高度
        
        # 状态跟踪变量
        self.barrier_detected_once = False    # 是否曾经检测到过限高杆
        self.post_barrier_distance = "50"    # 通过限高杆后发布的固定距离
        
        # 新增：检测框高度跟踪变量
        self.last_detection_height = None     # 上一次检测的框高度
        self.height_drop_threshold = 100      # 高度下降阈值（像素）
        self.height_triggered = False         # 是否因高度下降触发发布
        
        # 新增：红线位置跟踪变量
        self.position_threshold = 40          # 红线位置阈值（像素，y坐标）
        self.position_triggered = False       # 是否因红线位置过高触发发布
        
        # 新增：触发后状态
        self.publish_triggered = False        # 是否已触发发布模式（任一条件满足）
    
    def detect_red_barrier(self, image):
        """改进的红色限高杆检测 - 多ROI策略"""
        height, width = image.shape[:2]
        
        # 先用全图快速扫描确定大致区域
        best_detection = None
        best_roi_config = None
        
        # 按优先级尝试不同ROI配置 - 极近距离优先
        roi_priority = ['very_near', 'near', 'medium', 'far']
        
        for roi_name in roi_priority:
            roi_config = self.roi_configs[roi_name]
            detection = self._detect_in_roi(image, roi_config, roi_name)
            
            if detection and (best_detection is None or detection['confidence'] > best_detection['confidence']):
                best_detection = detection
                best_roi_config = roi_name
                
                # 如果检测置信度足够高，直接返回
                if detection['confidence'] > 0.7:
                    break
        
        if best_detection:
            # 生成用于可视化的掩码
            roi_config = self.roi_configs[best_roi_config]
            roi_top = int(height * roi_config['top'])
            roi_bottom = int(height * roi_config['bottom'])
            roi_image = image[roi_top:roi_bottom, :]
            
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask3 = cv2.inRange(hsv, self.lower_orange_red, self.upper_orange_red)
            mask4 = cv2.inRange(hsv, self.lower_dark_red, self.upper_dark_red)
            mask = cv2.bitwise_or(cv2.bitwise_or(mask1, mask2), cv2.bitwise_or(mask3, mask4))
            
            kernel_small = np.ones((2, 2), np.uint8)
            kernel_large = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
            
            best_detection['roi_config'] = best_roi_config
            return best_detection, mask
        
        # 如果没有检测到，返回空掩码
        empty_mask = np.zeros((height, width), dtype=np.uint8)
        return None, empty_mask
        
    def get_barrier_distance(self, image):
        """
        获取限高杆距离信息（包含新的触发条件）
        
        触发条件（满足其一即可）：
        1. 检测框高度突然降低100像素
        2. 红线位置小于40像素（接近图片顶部）
        
        触发后直接发布"50"，不再进行检测
        
        返回: (是否发布距离, 距离值, 检测结果, 掩码)
        """
        # 如果已经触发发布模式，直接返回发布状态
        if self.publish_triggered:
            return True, self.post_barrier_distance, None, np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        
        # 执行检测
        detection_result, mask = self.detect_red_barrier(image)
        
        if detection_result:
            bbox = detection_result['bbox']
            current_height = bbox[3]  # 当前检测框高度
            red_top_y = bbox[1]       # 红线顶部y坐标
            
            # 记录状态
            self.barrier_detected_once = True
            
            # 检查触发条件1：检测框高度突然下降
            if self.last_detection_height is not None:
                height_drop = self.last_detection_height - current_height
                if height_drop >= self.height_drop_threshold:
                    self.height_triggered = True
                    self.publish_triggered = True
                    print(f"🔥 触发条件1：检测框高度下降 {height_drop}像素 (阈值: {self.height_drop_threshold})")
                    print(f"   上次高度: {self.last_detection_height}, 当前高度: {current_height}")
                    return True, self.post_barrier_distance, detection_result, mask
            
            # 检查触发条件2：红线位置过高（接近图片顶部）
            if red_top_y <= self.position_threshold:
                self.position_triggered = True
                self.publish_triggered = True
                print(f"🔥 触发条件2：红线位置过高 y={red_top_y} (阈值: {self.position_threshold})")
                return True, self.post_barrier_distance, detection_result, mask
            
            # 更新上次检测高度
            self.last_detection_height = current_height
            
            # 未触发任何条件，仅记录状态，不发布距离
            return False, None, detection_result, mask
        
        else:
            # 未检测到红线，如果之前曾经检测到过且已触发，继续发布
            if self.barrier_detected_once and self.publish_triggered:
                return True, self.post_barrier_distance, None, mask
            
            # 如果从未检测到过，不发布距离
            return False, None, None, mask
    
    def _detect_in_roi(self, image, roi_config, roi_name):
        """在指定ROI中检测限高杆"""
        height, width = image.shape[:2]
        
        # 计算ROI区域
        roi_top = int(height * roi_config['top'])
        roi_bottom = int(height * roi_config['bottom'])
        roi_image = image[roi_top:roi_bottom, :]
        
        if roi_image.size == 0:
            return None
        
        # HSV转换
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        
        # 多重红色掩码组合 - 扩展范围
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask3 = cv2.inRange(hsv, self.lower_orange_red, self.upper_orange_red)
        mask4 = cv2.inRange(hsv, self.lower_dark_red, self.upper_dark_red)
        
        # 组合所有掩码
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.bitwise_or(mask, mask3)
        mask = cv2.bitwise_or(mask, mask4)
        
        # 形态学操作
        kernel_small = np.ones((2, 2), np.uint8)
        kernel_large = np.ones((3, 3), np.uint8)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        
        # 查找轮廓
        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_result) == 3:
            _, contours, _ = contours_result
        else:
            contours, _ = contours_result
        
        # 过滤轮廓
        valid_contours = []
        for c in contours:
            if c is not None and len(c) > 4:
                valid_contours.append(c)
        contours = valid_contours
        
        if not contours:
            return None
        
        # 根据ROI调整检测参数
        min_area = self.min_contour_area
        if roi_name == 'very_near':
            min_area = self.min_contour_area * 2  # 极近距离需要更大面积避免误检
        elif roi_name == 'near':
            min_area = self.min_contour_area * 1.5  # 近距离稍微提高要求
        elif roi_name == 'far':
            min_area = self.min_contour_area * 0.5  # 远距离放宽要求
        
        # 改进的轮廓分析
        best_contour = None
        best_score = 0
        best_bbox = None
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            # 尺寸过滤
            if (w < self.min_width or w > self.max_width or 
                h < self.min_height or h > self.max_height):
                continue
            
            aspect_ratio = h / w if w > 0 else 0
            
            # 根据ROI调整高宽比要求
            min_ratio, max_ratio = self.min_aspect_ratio, self.max_aspect_ratio
            if roi_name == 'very_near':
                # 极近距离时限高杆可能呈现各种形状，但不能太极端
                min_ratio = 0.02
                max_ratio = 50.0
            elif roi_name == 'near':
                # 近距离时限高杆可能很矮很宽
                min_ratio = 0.05
                max_ratio = 20.0
            
            if aspect_ratio < min_ratio or aspect_ratio > max_ratio:
                continue
            
            # 计算多维度分数
            bbox_area = w * h
            fill_ratio = area / bbox_area if bbox_area > 0 else 0
            
            # 面积分数
            area_score = min(area / 1000, 1.0)
            
            # 高宽比分数 - 根据ROI调整理想比例
            if roi_name == 'very_near':
                ideal_ratio = 1.0  # 极近距离任意形状都可接受
                ratio_score = 0.8  # 固定给高分
            elif roi_name == 'near':
                ideal_ratio = 0.8
                ratio_score = max(0, 1.0 - abs(aspect_ratio - ideal_ratio) / ideal_ratio)
            else:
                ideal_ratio = 1.5
                ratio_score = max(0, 1.0 - abs(aspect_ratio - ideal_ratio) / ideal_ratio)
            
            # 填充度分数
            fill_score = min(fill_ratio * 2, 1.0)
            
            # 位置分数
            center_x = x + w // 2
            roi_center = roi_image.shape[1] // 2
            position_score = 1.0 - abs(center_x - roi_center) / roi_center
            
            # ROI奖励分数 - 极近距离获得最高奖励
            roi_bonus = {'very_near': 1.5, 'near': 1.3, 'medium': 1.1, 'far': 1.0}.get(roi_name, 1.0)
            
            # 综合评分
            total_score = (area_score * 0.25 + ratio_score * 0.2 + fill_score * 0.25 + 
                          position_score * 0.15 + 0.15) * roi_bonus
            
            if total_score > best_score and total_score > self.detection_threshold:
                best_score = total_score
                best_contour = contour
                best_bbox = (x, y + roi_top, w, h)  # 转换到原图坐标
        
        if best_contour is None:
            return None
        
        # 估算距离
        distance = self.estimate_distance(best_bbox[3])
        
        return {
            'distance': distance,
            'confidence': best_score,
            'bbox': best_bbox,
            'contour': best_contour,
            'roi_top': roi_top,
            'roi_name': roi_name
        }
    
    def estimate_distance(self, object_height_pixels):
        """改进的距离估算"""
        if object_height_pixels <= 0:
            return 10.0
        
        distance = (self.reference_height_pixels * self.reference_distance) / object_height_pixels
        return max(0.3, min(distance, 15.0))  # 0.3米到15米
    
    def visualize_detection(self, image, detection_result, mask):
        """改进的可视化"""
        debug_image = image.copy()
        height, width = image.shape[:2]
        
        # 显示所有ROI配置区域
        colors = {
            'very_near': (0, 255, 0),     # 绿色 - 极近距离
            'near': (0, 255, 255),        # 黄色 - 近距离
            'medium': (255, 255, 0),      # 青色 - 中距离  
            'far': (255, 0, 255),         # 紫色 - 远距离
        }
        
        for roi_name, roi_config in self.roi_configs.items():
            roi_top = int(height * roi_config['top'])
            roi_bottom = int(height * roi_config['bottom'])
            color = colors.get(roi_name, (255, 255, 255))
            cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), color, 1)
            cv2.putText(debug_image, roi_name, (10, roi_top + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 高亮当前使用的ROI
        if detection_result and 'roi_name' in detection_result:
            roi_name = detection_result['roi_name']
            roi_config = self.roi_configs[roi_name]
            roi_top = int(height * roi_config['top'])
            roi_bottom = int(height * roi_config['bottom'])
            color = colors.get(roi_name, (255, 255, 255))
            cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), color, 3)
            cv2.putText(debug_image, f"ACTIVE: {roi_name}", (width - 150, roi_top + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # 绘制红色区域
        if mask is not None and detection_result:
            roi_config = self.roi_configs[detection_result['roi_name']]
            roi_top = int(height * roi_config['top'])
            roi_bottom = int(height * roi_config['bottom'])
            
            overlay = debug_image.copy()
            overlay[roi_top:roi_bottom, :][mask > 0] = [0, 0, 255]
            debug_image = cv2.addWeighted(debug_image, 0.7, overlay, 0.3, 0)
        
        # 绘制检测结果
        if detection_result:
            x, y, w, h = detection_result['bbox']
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # 绘制十字中心
            center_x, center_y = x + w//2, y + h//2
            cv2.line(debug_image, (center_x-15, center_y), (center_x+15, center_y), (0, 255, 0), 2)
            cv2.line(debug_image, (center_x, center_y-15), (center_x, center_y+15), (0, 255, 0), 2)
            
            distance = detection_result['distance']
            confidence = detection_result['confidence']
            roi_name = detection_result.get('roi_name', 'unknown')
            
            cv2.putText(debug_image, f"DETECTED ({roi_name})!", (x, y - 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(debug_image, f"Distance: {distance:.2f}m", (x, y - 55), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_image, f"Confidence: {confidence:.2f}", (x, y - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_image, f"Size: {w}x{h}", (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            result_text = f"DETECTED: {distance:.2f}m (conf: {confidence:.2f}) ROI: {roi_name}"
            color = (0, 255, 0)
        else:
            result_text = "NO BARRIER DETECTED"
            color = (0, 0, 255)
        
        # 状态信息
        cv2.putText(debug_image, result_text, (10, height - 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # 参数信息
        param_text = f"Area:{self.min_contour_area}+ Ratio:{self.min_aspect_ratio:.1f}-{self.max_aspect_ratio:.1f}"
        cv2.putText(debug_image, param_text, (10, height - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # HSV范围信息
        hsv_text = f"HSV: Red[{self.lower_red1[0]}-{self.upper_red1[0]}] [{self.lower_red2[0]}-{self.upper_red2[0]}] Orange[{self.lower_orange_red[0]}-{self.upper_orange_red[0]}] Dark[{self.lower_dark_red[0]}-{self.upper_dark_red[0]}]"
        cv2.putText(debug_image, hsv_text, (10, height - 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # ROI策略说明
        strategy_text = "ROI Strategy: very_near(full), near(80%), medium(mid), far(center)"
        cv2.putText(debug_image, strategy_text, (10, height - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return debug_image
    
    def test_image(self, image_path):
        """测试单张图片"""
        if not os.path.exists(image_path):
            print(f"图片不存在: {image_path}")
            return
        
        image = cv2.imread(image_path)
        if image is None:
            print(f"无法读取图片: {image_path}")
            return
        
        print(f"测试图片: {image_path}")
        print(f"图片尺寸: {image.shape}")
        
        # 使用新的状态逻辑检测
        should_publish, distance_value, detection_result, mask = self.get_barrier_distance(image)
        
        # 输出结果
        if should_publish:
            if self.publish_triggered:
                if self.height_triggered:
                    print(f"🔥 触发发布模式 (高度下降) - 持续发布距离: {distance_value}")
                elif self.position_triggered:
                    print(f"🔥 触发发布模式 (红线位置过高) - 持续发布距离: {distance_value}")
                else:
                    print(f"🔥 触发发布模式 - 持续发布距离: {distance_value}")
            else:
                print(f"📤 发布固定距离: {distance_value}")
        elif detection_result:
            bbox = detection_result['bbox']
            red_top_y = bbox[1]  # 红线顶部y坐标
            current_height = bbox[3]  # 检测框高度
            image_height = image.shape[0]
            
            print(f"✅ 检测到红色限高杆！")
            print(f"   实际距离: {detection_result['distance']:.2f} 米")
            print(f"   置信度: {detection_result['confidence']:.3f}")
            print(f"   位置: {detection_result['bbox']}")
            print(f"   红线高度: y={red_top_y} (图片高度: {image_height})")
            print(f"   检测框高度: {current_height}像素")
            if self.last_detection_height is not None:
                height_change = current_height - self.last_detection_height
                print(f"   高度变化: {height_change:+d}像素 (上次: {self.last_detection_height})")
            print(f"   🚫 未触发发布条件 (红线位置>{self.position_threshold}, 高度变化<{self.height_drop_threshold})")
        else:
            print("❌ 未检测到红色限高杆")
        
        # 显示状态
        status_parts = [f"barrier_detected_once={self.barrier_detected_once}"]
        if self.publish_triggered:
            status_parts.append("publish_triggered=True")
            if self.height_triggered:
                status_parts.append("height_triggered=True")
            if self.position_triggered:
                status_parts.append("position_triggered=True")
        print(f"   状态: {', '.join(status_parts)}")
        
        # 可视化
        debug_image = self.visualize_detection(image, detection_result, mask)
        
        # 添加状态信息到调试图像
        status_text = f"State: detected_once={self.barrier_detected_once}"
        if should_publish:
            status_text += f" | Publishing: {distance_value}m"
        else:
            status_text += " | Not Publishing"
        
        cv2.putText(debug_image, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 保存调试图像
        debug_path = image_path.replace('.png', '_debug.png')
        cv2.imwrite(debug_path, debug_image)
        print(f"调试图像已保存: {debug_path}")
        
        # 显示图片
        cv2.imshow('Original', cv2.resize(image, (800, 600)))
        if mask is not None:
            cv2.imshow('Mask', cv2.resize(mask, (800, 600)))
        else:
            # 创建空白掩码用于显示
            blank_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
            cv2.imshow('Mask', cv2.resize(blank_mask, (800, 600)))
        cv2.imshow('Detection Result', cv2.resize(debug_image, (800, 600)))
        
        print("按任意键继续...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def batch_test(self, image_folder):
        """批量测试图片 - 模拟连续帧序列"""
        if not os.path.exists(image_folder):
            print(f"文件夹不存在: {image_folder}")
            return
        
        image_files = []
        for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
            import glob
            image_files.extend(glob.glob(os.path.join(image_folder, ext)))
        
        image_files.sort()
        print(f"找到 {len(image_files)} 张图片")
        print("模拟连续帧序列测试...")
        
        detected_count = 0
        published_count = 0
        
        for i, image_path in enumerate(image_files, 1):
            print(f"\n--- 帧 {i}/{len(image_files)}: {os.path.basename(image_path)} ---")
            
            image = cv2.imread(image_path)
            if image is None:
                continue
            
            # 使用新的状态逻辑
            should_publish, distance_value, detection_result, mask = self.get_barrier_distance(image)
            
            if detection_result:
                detected_count += 1
                bbox = detection_result['bbox']
                red_top_y = bbox[1]  # 红线顶部y坐标
                image_height = image.shape[0]
                print(f"✅ 检测到红色限高杆! 距离: {detection_result['distance']:.2f}m, 置信度: {detection_result['confidence']:.3f}")
                print(f"   红线高度: y={red_top_y} (图片高度: {image_height})")
                print(f"🚫 不发布距离 (仅记录状态)")
            
            if should_publish:
                published_count += 1
                print(f"📤 持续发布固定距离: {distance_value}m (曾经检测到过)")
            elif not detection_result:
                print("🚫 不发布距离 (从未检测到过)")
            
            print(f"状态: detected_once={self.barrier_detected_once}")
        
        print(f"\n=== 批量测试完成 ===")
        print(f"总帧数: {len(image_files)}")
        print(f"检测到限高杆的帧数: {detected_count}")
        print(f"发布距离的帧数: {published_count}")
        print(f"检测率: {detected_count/len(image_files)*100:.1f}%")
        print(f"发布率: {published_count/len(image_files)*100:.1f}%")
        print(f"最终状态: barrier_detected_once={self.barrier_detected_once}")
    
    def reset_state(self):
        """重置状态 - 用于测试不同序列"""
        self.barrier_detected_once = False
        self.last_detection_height = None
        self.height_triggered = False
        self.position_triggered = False
        self.publish_triggered = False
        print("状态已重置: 所有触发条件和检测状态已清除")


def main():
    if len(sys.argv) < 2:
        print("用法:")
        print("  单张测试: python3 red_barrier_detector_improved.py <图片路径>")
        print("  批量测试: python3 red_barrier_detector_improved.py <图片文件夹>")
        print("  重置测试: python3 red_barrier_detector_improved.py reset <图片文件夹>")
        print("例如:")
        print("  python3 red_barrier_detector_improved.py img3/10.png")
        print("  python3 red_barrier_detector_improved.py img3/")
        print("  python3 red_barrier_detector_improved.py reset img3/")
        return
    
    tester = ImprovedRedBarrierTester()
    
    if sys.argv[1] == "reset" and len(sys.argv) > 2:
        # 重置状态后进行批量测试
        path = sys.argv[2]
        tester.reset_state()
        if os.path.isdir(path):
            tester.batch_test(path)
        else:
            print(f"路径不存在或不是文件夹: {path}")
    else:
        path = sys.argv[1]
        if os.path.isfile(path):
            tester.test_image(path)
        elif os.path.isdir(path):
            tester.batch_test(path)
        else:
            print(f"路径不存在: {path}")


if __name__ == '__main__':
    main()