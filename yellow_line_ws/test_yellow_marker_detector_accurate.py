#!/usr/bin/env python3

"""
精确的黄色标志物检测器测试脚本
目标：图片1-10每张有1个圆，图片11没有圆
避免误检，提高检测精度
"""

import cv2
import numpy as np
import os
import sys
from typing import Optional, Tuple

class YellowMarkerDetectorAccurate:
    def __init__(self):
        # 放宽黄色范围以适应不同光照条件
        self.lower_yellow = np.array([15, 50, 50])     # 进一步放宽色调、饱和度和亮度下限
        self.upper_yellow = np.array([35, 255, 255])   # 扩大色调上限
        
        # 检测参数 - 针对天花板上的圆形标志物
        self.roi_top_ratio = 0.0       # 从顶部开始
        self.roi_bottom_ratio = 0.6    # 扩大到60%区域覆盖更多天花板
        self.valid_circle_y_ratio = 0.5  # 只接受ROI前50%区域内的圆形（真正的天花板区域）
        
        # 霍夫圆检测参数 - 放宽参数提高检测率
        self.min_radius = 5            # 减小最小半径
        self.max_radius = 150          # 增加最大半径  
        self.min_dist = 20             # 减小圆心间最小距离
        self.param1 = 20               # 进一步降低边缘检测阈值
        self.param2 = 10               # 进一步降低累积阈值
        
        # 面积过滤参数 - 适应更宽的黄色范围
        self.min_area = 200            # 最小黄色区域面积
        self.max_area = 15000          # 增加最大黄色区域面积以适应更宽的HSV范围
        
    def detect_yellow_marker(self, image: np.ndarray) -> Optional[Tuple[float, Tuple[int, int, int]]]:
        """
        精确检测黄色标志物 - 避免误检
        """
        try:
            print(f'开始精确检测 - 图像形状: {image.shape}')
            height, width = image.shape[:2]
            
            # 限制ROI区域到天花板部分
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            roi_image = image[roi_top:roi_bottom, :]
            print(f'天花板ROI区域: ({roi_top}, {roi_bottom}), 形状: {roi_image.shape}')
            
            # HSV转换
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建严格的黄色掩码
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            yellow_pixels = np.sum(mask > 0)
            print(f'黄色像素数: {yellow_pixels}')
            
            # 如果黄色像素太少，直接返回
            if yellow_pixels < self.min_area:
                print("黄色像素数太少，无标志物")
                return None
            
            # 如果黄色像素太多，可能是误检黄线
            if yellow_pixels > self.max_area:
                print("黄色像素数太多，可能是地面黄线，跳过")
                return None
            
            # 形态学操作 - 去噪声
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 使用严格的霍夫圆检测参数
            circles = cv2.HoughCircles(
                mask,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=self.min_dist,
                param1=self.param1,
                param2=self.param2, 
                minRadius=self.min_radius,
                maxRadius=self.max_radius
            )
            
            if circles is not None:
                circles = np.uint16(np.around(circles))
                print(f'检测到{len(circles[0])}个候选圆形')
                
                # 只返回最佳的一个圆（最大的或最圆的）
                best_circle = None
                best_score = 0
                
                for circle in circles[0, :]:
                    x, y, radius = circle
                    
                    # 验证圆形的有效性（包括位置过滤）
                    roi_height = roi_image.shape[0]
                    if not self.validate_circle(mask, x, y, radius, roi_height):
                        continue
                    
                    # 计算圆形质量分数（半径 + 圆形度）
                    circularity = self.calculate_circularity(mask, x, y, radius)
                    score = radius * circularity  # 半径越大、越圆的分数越高
                    
                    print(f'圆形: 位置({x},{y}), 半径{radius}, 圆形度{circularity:.2f}, 分数{score:.1f}')
                    
                    if score > best_score:
                        best_score = score
                        best_circle = circle
                
                if best_circle is not None:
                    x, y, radius = best_circle
                    distance = self.calculate_distance_from_circle(radius)
                    print(f'最佳圆形: 位置({x},{y}), 半径{radius}, 距离{distance:.2f}m')
                    return (distance, (x, y, radius))
            
            print("未检测到有效的圆形标志物")
            return None
            
        except Exception as e:
            print(f'检测过程出错: {str(e)}')
            return None
    
    def validate_circle(self, mask: np.ndarray, x: int, y: int, radius: int, roi_height: int) -> bool:
        """验证圆形的有效性 - 包括位置和填充度验证"""
        try:
            # 添加Y坐标位置过滤：只接受ROI前50%区域内的圆形
            max_valid_y = int(roi_height * self.valid_circle_y_ratio)
            if y > max_valid_y:
                print(f'圆形Y坐标{y}超过有效区域{max_valid_y}，跳过')
                return False
            
            # 检查圆形是否在掩码范围内
            h, w = mask.shape
            x, y, radius = int(x), int(y), int(radius)  # 确保为整数类型
            if x - radius < 0 or x + radius >= w or y - radius < 0 or y + radius >= h:
                return False
            
            # 检查圆形区域内的像素填充度
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (x, y), radius, 255, -1)
            
            intersection = cv2.bitwise_and(mask, circle_mask)
            circle_area = np.pi * radius * radius
            filled_area = np.sum(intersection > 0)
            fill_ratio = filled_area / circle_area
            
            # 要求至少50%的填充度
            return fill_ratio > 0.5
            
        except:
            return False
    
    def calculate_circularity(self, mask: np.ndarray, x: int, y: int, radius: int) -> float:
        """计算圆形的圆形度（0-1，1为完美圆形）"""
        try:
            # 创建圆形模板
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (x, y), radius, 255, -1)
            
            # 计算重叠度
            intersection = cv2.bitwise_and(mask, circle_mask)
            union = cv2.bitwise_or(mask, circle_mask)
            
            intersection_area = np.sum(intersection > 0)
            union_area = np.sum(union > 0)
            
            if union_area == 0:
                return 0.0
            
            # IoU作为圆形度指标
            return intersection_area / union_area
            
        except:
            return 0.0
    
    def calculate_distance_from_circle(self, pixel_radius: int) -> float:
        """根据像素半径计算距离"""
        focal_length_pixels = 600  # 假设焦距
        real_diameter = 0.2        # 假设真实直径20cm
        
        if pixel_radius > 0:
            distance = (real_diameter * focal_length_pixels) / (2 * pixel_radius)
            return max(0.1, min(distance, 10.0))
        return 10.0
    
    def create_debug_image(self, image: np.ndarray, detection_result) -> np.ndarray:
        """创建调试图像"""
        debug_image = image.copy()
        height, width = image.shape[:2]
        
        # 绘制ROI区域
        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        cv2.rectangle(debug_image, (0, roi_top), (width, roi_bottom), (255, 255, 0), 2)
        cv2.putText(debug_image, f"Ceiling ROI (0% - {int(self.roi_bottom_ratio*100)}%)", 
                   (10, roi_top + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # 在ROI区域检测黄色
        roi_image = image[roi_top:roi_bottom, :]
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 显示黄色区域
        yellow_overlay = debug_image.copy()
        yellow_overlay[roi_top:roi_bottom, :][mask > 0] = [0, 255, 255]
        debug_image = cv2.addWeighted(debug_image, 0.8, yellow_overlay, 0.2, 0)
        
        # 显示黄色像素统计
        yellow_pixels = np.sum(mask > 0)
        cv2.putText(debug_image, f"Yellow pixels: {yellow_pixels} (min: {self.min_area}, max: {self.max_area})", 
                   (10, roi_bottom + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 显示所有候选圆形
        circles = cv2.HoughCircles(
            mask, cv2.HOUGH_GRADIENT, dp=1, minDist=self.min_dist,
            param1=self.param1, param2=self.param2, 
            minRadius=self.min_radius, maxRadius=self.max_radius
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i, circle in enumerate(circles[0, :]):
                x, y, radius = circle
                y_adjusted = y + roi_top
                
                # 绘制候选圆形（浅蓝色）
                cv2.circle(debug_image, (x, y_adjusted), radius, (255, 255, 128), 2)
                cv2.putText(debug_image, f"C{i+1}", (x + 10, y_adjusted - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 128), 1)
        
        # 绘制最终检测结果
        if detection_result:
            distance, circle = detection_result
            x, y, radius = circle
            y_adjusted = y + roi_top
            
            # 绘制最终圆形（红色）
            cv2.circle(debug_image, (x, y_adjusted), radius, (0, 0, 255), 3)
            cv2.circle(debug_image, (x, y_adjusted), 5, (0, 0, 255), -1)
            
            # 显示信息
            cv2.putText(debug_image, f"DETECTED: D={distance:.2f}m, R={radius}px", 
                       (x + 10, y_adjusted - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            result_text = f"SUCCESS: 1 Yellow Marker Detected"
            color = (0, 255, 0)
        else:
            result_text = f"NO MARKER: Yellow pixels = {yellow_pixels}"
            color = (0, 0, 255)
        
        cv2.putText(debug_image, result_text, (10, height - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        return debug_image
    
    def test_single_image(self, image_path: str, save_debug: bool = True) -> bool:
        """测试单张图片"""
        print(f"\n{'='*50}")
        print(f"测试图片: {os.path.basename(image_path)}")
        print(f"{'='*50}")
        
        # 读取图片
        image = cv2.imread(image_path)
        if image is None:
            print(f"❌ 无法读取图片: {image_path}")
            return False
        
        print(f"图片尺寸: {image.shape}")
        
        # 检测
        detection_result = self.detect_yellow_marker(image)
        
        # 输出结果
        if detection_result:
            distance, circle = detection_result
            x, y, radius = circle
            print(f"✅ 检测到黄色标志物:")
            print(f"   位置: ({x}, {y})")
            print(f"   半径: {radius} 像素")
            print(f"   距离: {distance:.2f} 米")
        else:
            print("❌ 未检测到黄色标志物")
        
        # 保存调试图像
        if save_debug:
            debug_image = self.create_debug_image(image, detection_result)
            
            filename = os.path.basename(image_path)
            name, ext = os.path.splitext(filename)
            debug_path = f"test_results/{name}_accurate_debug{ext}"
            
            os.makedirs("test_results", exist_ok=True)
            cv2.imwrite(debug_path, debug_image)
            print(f"📁 调试图像已保存: {debug_path}")
        
        return detection_result is not None
    
    def test_all_images(self, image_folder: str = "img6"):
        """测试所有图片"""
        if not os.path.exists(image_folder):
            print(f"❌ 图片文件夹不存在: {image_folder}")
            return
        
        print(f"🔍 精确测试黄色标志物检测器")
        print(f"📁 测试文件夹: {image_folder}")
        print(f"🎯 期望结果: 图片1-10各有1个圆，图片11无圆")
        print("=" * 80)
        
        # 获取图片文件
        image_files = []
        for file in os.listdir(image_folder):
            if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                image_files.append(file)
        
        image_files.sort(key=lambda x: int(x.split('.')[0]))  # 按数字排序
        
        if not image_files:
            print("❌ 文件夹中没有找到图片文件")
            return
        
        # 统计结果
        total_images = len(image_files)
        detected_count = 0
        results = []
        expected_results = {f"{i}.png": True for i in range(1, 11)}  # 1-10应该有圆
        expected_results["11.png"] = False  # 11没有圆
        
        # 测试每张图片
        for i, filename in enumerate(image_files, 1):
            image_path = os.path.join(image_folder, filename)
            detected = self.test_single_image(image_path)
            results.append((filename, detected))
            
            if detected:
                detected_count += 1
            
            # 检查是否符合预期
            expected = expected_results.get(filename, False)
            status = "✅ 正确" if detected == expected else "❌ 错误"
            print(f"📊 {filename}: {'检测到' if detected else '未检测'} | 期望: {'有圆' if expected else '无圆'} | {status}")
        
        # 输出总结
        print("\n" + "=" * 80)
        print(f"🎯 精确检测测试完成!")
        print(f"📊 总图片数: {total_images}")
        print(f"🔍 检测到标志物: {detected_count}")
        print(f"📁 调试图像保存在: test_results/ 文件夹")
        
        # 准确率计算
        correct_count = 0
        for filename, detected in results:
            expected = expected_results.get(filename, False)
            if detected == expected:
                correct_count += 1
        
        accuracy = correct_count / total_images * 100
        print(f"🎯 检测准确率: {correct_count}/{total_images} = {accuracy:.1f}%")
        
        print(f"\n📋 详细结果:")
        for filename, detected in results:
            expected = expected_results.get(filename, False)
            status = "✅" if detected == expected else "❌"
            print(f"   {filename}: {'检测到' if detected else '未检测'} (期望: {'有圆' if expected else '无圆'}) {status}")

def main():
    """主函数"""
    print("🎯 精确黄色标志物检测器测试程序")
    print("目标: 图片1-9各检测1个圆，图片10,11检测0个圆")
    print("=" * 80)
    
    tester = YellowMarkerDetectorAccurate()
    
    if len(sys.argv) > 1:
        # 测试单张图片
        image_path = sys.argv[1]
        tester.test_single_image(image_path)
    else:
        # 测试所有图片
        tester.test_all_images("img6")

if __name__ == "__main__":
    main()