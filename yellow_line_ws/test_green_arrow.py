#!/usr/bin/env python3
"""
绿色箭头检测器测试脚本

功能描述：
    - 使用静态图片测试绿色箭头检测功能
    - 可视化检测过程和结果
    - 不依赖ROS环境

使用方法：
    python test_green_arrow.py [图片路径]
"""

import cv2
import numpy as np
import os
import sys
from typing import Optional


class GreenArrowTester:
    def __init__(self):
        # 绿色检测参数 - HSV色彩空间
        self.lower_green = np.array([35, 50, 50])   # 绿色下限
        self.upper_green = np.array([85, 255, 255]) # 绿色上限
        
        # 检测参数
        self.min_contour_area = 1000  # 最小轮廓面积
        self.roi_ratio = 0.7  # ROI区域比例（中间60%区域）
        
        print(f'初始化绿色箭头检测器')
        print(f'HSV绿色范围: {self.lower_green} - {self.upper_green}')
        print(f'最小轮廓面积: {self.min_contour_area}')
    
    def detect_arrow_direction(self, image: np.ndarray) -> Optional[str]:
        """
        检测绿色箭头方向
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            箭头方向: 'left', 'right' 或 None
        """
        try:
            height, width = image.shape[:2]
            print(f'图像尺寸: {width}x{height}')
            
            # 提取ROI区域
            roi = {
                'y1': int(height * 0.3),  # 从高度 30% 开始
                'y2': int(height * 0.7),  # 到高度 70%
                'x1': int(width * 0.3),   # 左边裁掉 30%
                'x2': int(width * 0.7),   # 右边裁掉 30%
            }
            roi_image = image[roi['y1']:roi['y2'], roi['x1']:roi['x2']]
            roi_width = roi_image.shape[1]
            roi_height = roi_image.shape[0]
            print(f'ROI区域: ({roi["x1"]}, {roi["y1"]}) 到 ({roi["x2"]}, {roi["y2"]})')
            print(f'ROI尺寸: {roi_width}x{roi_height}')
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建绿色掩码
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            
            # 统计绿色像素数量
            green_pixels = np.sum(mask > 0)
            print(f'检测到绿色像素数量: {green_pixels}')
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                print('未检测到绿色轮廓')
                return None
            
            print(f'找到 {len(contours)} 个轮廓')
            
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)
            print(f'最大轮廓面积: {contour_area}')
            
            # 检查轮廓面积
            if contour_area < self.min_contour_area:
                print(f'轮廓面积 {contour_area} 小于最小值 {self.min_contour_area}')
                return None
            
            # 分析箭头方向
            direction = self.analyze_arrow_direction(largest_contour, roi_width)
            
            return direction
            
        except Exception as e:
            print(f'检测箭头方向时出错: {str(e)}')
            return None
    
    def analyze_arrow_direction(self, contour: np.ndarray, roi_width: int) -> Optional[str]:
        """
        分析轮廓的箭头方向
        
        Args:
            contour: 输入轮廓
            roi_width: ROI区域宽度
            
        Returns:
            箭头方向: 'left', 'right' 或 None
        """
        try:
            # 计算轮廓的质心
            M = cv2.moments(contour)
            if M["m00"] == 0:
                print('无法计算轮廓质心')
                return None
            
            cx = int(M["m10"] / M["m00"])  # 质心X坐标
            cy = int(M["m01"] / M["m00"])  # 质心Y坐标
            print(f'轮廓质心: ({cx}, {cy})')
            
            # 计算轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            print(f'边界框: ({x}, {y}), 尺寸: {w}x{h}')
            
            # 分析轮廓形状特征来判断箭头方向
            # 方法1：分析质心相对于边界框的位置
            bbox_center_x = x + w / 2
            centroid_offset = cx - bbox_center_x
            print(f'边界框中心X: {bbox_center_x}, 质心偏移: {centroid_offset}')
            
            direction_score = 0  # 正数表示右箭头，负数表示左箭头
            
            # 基于质心偏移判断
            if abs(centroid_offset) > w * 0.1:  # 至少偏移10%
                if centroid_offset > 0:
                    direction_score += 1  # 质心偏右，可能是右箭头
                    print('质心偏右 -> 右箭头倾向')
                else:
                    direction_score -= 1  # 质心偏左，可能是左箭头
                    print('质心偏左 -> 左箭头倾向')
            
            # 基于轮廓最左和最右点分析
            leftmost = tuple(contour[contour[:, :, 0].argmin()][0])
            rightmost = tuple(contour[contour[:, :, 0].argmax()][0])
            print(f'轮廓最左点: {leftmost}, 最右点: {rightmost}')
            
            # 分析边缘形状
            left_edge_y = leftmost[1]
            right_edge_y = rightmost[1]
            center_y = y + h / 2
            
            left_distance = abs(left_edge_y - center_y)
            right_distance = abs(right_edge_y - center_y)
            print(f'左边缘到中心距离: {left_distance}, 右边缘到中心距离: {right_distance}')
            
            # 如果左边缘更尖锐（Y坐标更接近中心），可能是左箭头
            if left_distance < right_distance:
                direction_score -= 1
                print('左边缘更尖锐 -> 左箭头倾向')
            else:
                direction_score += 1
                print('右边缘更尖锐 -> 右箭头倾向')
            
            print(f'方向得分: {direction_score}')
            
            # 根据分数判断方向
            if direction_score > 0:
                return 'right'
            elif direction_score < 0:
                return 'left'
            else:
                # 如果分数为0，使用简单的质心位置判断
                roi_center = roi_width / 2
                print(f'ROI中心: {roi_center}, 质心X: {cx}')
                if cx > roi_center:
                    return 'right'
                else:
                    return 'left'
                    
        except Exception as e:
            print(f'分析箭头方向时出错: {str(e)}')
            return None
    
    def create_debug_image(self, image: np.ndarray, direction: Optional[str]) -> np.ndarray:
        """创建调试可视化图像"""
        debug_image = image.copy()
        height, width = image.shape[:2]
        
        # 绘制ROI区域
        roi = {
            'y1': int(height * 0.3),  # 从高度 30% 开始
            'y2': int(height * 0.7),  # 到高度 70%
            'x1': int(width * 0.3),   # 左边裁掉 30%
            'x2': int(width * 0.7),   # 右边裁掉 30%
        }
        cv2.rectangle(debug_image, (roi['x1'], roi['y1']), (roi['x2'], roi['y2']), (255, 255, 0), 2)
        cv2.putText(debug_image, "ROI", (roi['x1'] + 10, roi['y1'] + 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # 在ROI区域检测绿色
        roi_image = image[roi['y1']:roi['y2'], roi['x1']:roi['x2']]
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 在原图上绘制绿色区域
        green_overlay = debug_image.copy()
        green_overlay[roi['y1']:roi['y2'], roi['x1']:roi['x2']][mask > 0] = [0, 255, 0]
        debug_image = cv2.addWeighted(debug_image, 0.7, green_overlay, 0.3, 0)
        
        # 查找并绘制轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) >= self.min_contour_area:
                # 调整轮廓坐标到原图
                adjusted_contour = largest_contour.copy()
                adjusted_contour[:, :, 0] += roi['x1']
                adjusted_contour[:, :, 1] += roi['y1']
                
                # 绘制轮廓
                cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 255, 255), 3)
                
                # 绘制质心
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + roi['x1']
                    cy = int(M["m01"] / M["m00"]) + roi['y1']
                    cv2.circle(debug_image, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.putText(debug_image, f"Centroid({cx},{cy})", (cx + 15, cy), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 显示检测结果
        if direction:
            direction_text = f"Arrow Direction: {direction.upper()}"
            color = (0, 255, 0)
        else:
            direction_text = "No Arrow Detected"
            color = (0, 0, 255)
        
        cv2.putText(debug_image, direction_text, (10, height - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        # 显示参数信息
        param_text = f"HSV Range: {self.lower_green}-{self.upper_green}, Min Area: {self.min_contour_area}"
        cv2.putText(debug_image, param_text, (10, height - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return debug_image
    
    def test_image(self, image_path: str) -> None:
        """测试单张图片"""
        print(f'\n=== 测试图片: {image_path} ===')
        
        # 读取图片
        if not os.path.exists(image_path):
            print(f'错误: 图片文件不存在: {image_path}')
            return
        
        image = cv2.imread(image_path)
        if image is None:
            print(f'错误: 无法读取图片: {image_path}')
            return
        
        print(f'成功读取图片: {image_path}')
        
        # 检测箭头方向
        direction = self.detect_arrow_direction(image)
        
        # 创建可视化图像
        debug_image = self.create_debug_image(image, direction)
        
        # 显示结果
        if direction:
            print(f'\n✓ 检测结果: 绿色箭头方向为 {direction.upper()}')
        else:
            print(f'\n✗ 检测结果: 未检测到绿色箭头')
        
        # 显示图像
        cv2.imshow(f'Original - {os.path.basename(image_path)}', image)
        cv2.imshow(f'Detection Result - {os.path.basename(image_path)}', debug_image)
        
        print('\n按任意键继续下一张图片，按 ESC 退出...')
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        
        if key == 27:  # ESC键
            print('用户退出测试')
            sys.exit(0)


def main():
    # 创建测试器实例
    tester = GreenArrowTester()
    
    # 获取测试图片路径
    if len(sys.argv) > 1:
        # 命令行指定图片路径
        image_paths = sys.argv[1:]
    else:
        # 默认测试路径
        default_paths = [
            'test_results/',
            'img5/',
            './'
        ]
        
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
        image_paths = []
        
        # 查找所有图片文件
        for path in default_paths:
            if os.path.exists(path):
                if os.path.isdir(path):
                    for filename in os.listdir(path):
                        if any(filename.lower().endswith(ext) for ext in image_extensions):
                            image_paths.append(os.path.join(path, filename))
                elif os.path.isfile(path) and any(path.lower().endswith(ext) for ext in image_extensions):
                    image_paths.append(path)
        
        if not image_paths:
            print('未找到测试图片。请指定图片路径:')
            print('python test_green_arrow.py <图片路径1> [图片路径2] ...')
            return
        
        print(f'找到 {len(image_paths)} 张图片进行测试')
    
    # 逐一测试图片
    for i, image_path in enumerate(image_paths, 1):
        print(f'\n{"="*50}')
        print(f'测试进度: {i}/{len(image_paths)}')
        tester.test_image(image_path)
    
    print('\n测试完成！')


if __name__ == '__main__':
    main()