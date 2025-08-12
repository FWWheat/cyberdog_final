#!/usr/bin/env python3

"""
黄线检测器 - 用于检测黄色纵向平行线并判断机器人位置

功能描述：
    - 检测图像中的黄色纵向平行线
    - 判断机器人相对于两条黄线的位置（左、右、中间）
    - 支持鱼眼相机和RGB相机两种图像格式
    
使用方法：
    from yellow_line_detector import YellowLineDetector
    
    detector = YellowLineDetector()
    position = detector.detect_position(image)
    # 返回值: 'left', 'right', 'center'
"""

import cv2
import numpy as np
import os
from typing import Optional, Tuple, List


def safe_float(value):
    """安全的float转换，处理NumPy数组和标量"""
    if isinstance(value, (np.ndarray, list, tuple)):
        arr = np.array(value).flatten()
        if arr.size == 1:
            return float(arr[0])
        else:
            return float(np.mean(arr))
    return float(value)


def safe_compare(a, b, operator='<'):
    """安全的数值比较，避免数组比较错误"""
    a_val = safe_float(a)
    b_val = safe_float(b)
    
    if operator == '<':
        return a_val < b_val
    elif operator == '>':
        return a_val > b_val
    elif operator == '<=':
        return a_val <= b_val
    elif operator == '>=':
        return a_val >= b_val
    elif operator == '==':
        return a_val == b_val
    else:
        raise ValueError(f"Unsupported operator: {operator}")


class YellowLineDetector:
    """黄线检测器类"""
    
    def __init__(self, 
                 lower_yellow: Tuple[int, int, int] = (15, 50, 30),
                 upper_yellow: Tuple[int, int, int] = (35, 255, 255),
                 position_threshold: float = 0.05,
                 roi_bottom_ratio: float = 0.4):
        """
        初始化黄线检测器
        
        Args:
            lower_yellow: HSV黄色下限 (H, S, V) - 调整为更宽泛的黄色范围
            upper_yellow: HSV黄色上限 (H, S, V)
            position_threshold: 位置判断阈值（图像宽度的百分比）
            roi_bottom_ratio: ROI区域比例（图像下方多少比例）
        """
        self.lower_yellow = np.array(lower_yellow)
        self.upper_yellow = np.array(upper_yellow)
        self.position_threshold = position_threshold
        self.roi_bottom_ratio = roi_bottom_ratio
        
        # 形态学操作核
        self.morph_kernel = np.ones((3, 3), np.uint8)  # 减小核大小
        
        # 霍夫变换阈值
        self.hough_threshold = 50
    
    def detect_yellow_between_center(self, image: np.ndarray, roi_params: dict = None, 
                                   camera_type: str = 'rgb', threshold_params: dict = None) -> str:
        """
        两条黄线中间检测(ycy)：输入图片，输出左，右，中
        图片感兴趣区域，识别黄色区域，计算质心，计算位置偏移
        
        Args:
            image: 输入图像 (BGR格式)
            roi_params: ROI区域参数，如 {'top_ratio': 0.6, 'bottom_ratio': 0.4, 'left_ratio': 0.1, 'right_ratio': 0.9}
            camera_type: 相机类型 ('rgb', 'fisheye_left', 'fisheye_right')
            threshold_params: 阈值参数，如 {'position_threshold': 0.05, 'area_threshold': 500}
            
        Returns:
            位置字符串: 'left' (靠近左线), 'right' (靠近右线), 'center' (在中间)
        """
        if image is None:
            return 'center'
        
        # 获取阈值参数
        position_threshold = threshold_params.get('position_threshold', self.position_threshold) if threshold_params else self.position_threshold
        area_threshold = threshold_params.get('area_threshold', 500) if threshold_params else 500
        
        try:
            # 获取图像尺寸
            height, width = image.shape[:2]
            
            # 应用ROI参数
            if roi_params:
                roi_top = int(height * roi_params.get('top_ratio', 1 - self.roi_bottom_ratio))
                roi_bottom = int(height * roi_params.get('bottom_ratio', 1.0))
                roi_left = int(width * roi_params.get('left_ratio', 0.0))
                roi_right = int(width * roi_params.get('right_ratio', 1.0))
                roi_image = image[roi_top:roi_bottom, roi_left:roi_right]
                roi_width = roi_right - roi_left
            else:
                # 使用默认ROI区域（图像下方部分）
                roi_start = int(height * (1 - self.roi_bottom_ratio))
                roi_image = image[roi_start:, :]
                roi_width = width
            
            print(f"[黄线检测-YCY] 处理{camera_type}相机图像，ROI区域: {roi_image.shape}")
            print(f"[黄线检测-YCY] 使用阈值 - 位置阈值: {position_threshold}, 面积阈值: {area_threshold}")
            
            # 转换为HSV色彩空间并检测黄色
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
            mask = mask.astype(np.uint8)  # 确保是 8 位单通道

            # 基于轮廓的质心检测
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                 _, contours, hierarchy = contours_result
            else:
                contours, hierarchy = contours_result

            valid_contours = []
            for c in contours:
                if c is not None and len(c) > 0 and c.dtype in [np.int32, np.float32]:
                    valid_contours.append(c)
            print(f"[黄线检测-YCY] 检测到valid_contours {len(valid_contours)} 个轮廓")

            if valid_contours:
                contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)
                # 过滤符合面积阈值的轮廓，然后找到质心x最接近中点的轮廓
                centroids = []
                for contour in contours[:5]:
                    if cv2.contourArea(contour) > area_threshold:
                        M = cv2.moments(contour)
                        if safe_float(M["m00"]) != 0:
                            cx = float(M["m10"] / M["m00"])
                            centroids.append(cx)
                
                print(f"[黄线检测-YCY] 检测到 {len(centroids)} 个轮廓质心: {centroids}")

                # 检查所有质心是否都在一侧
                if len(centroids) > 0:
                    image_center = roi_width / 2
                    left_side_count = sum(1 for c in centroids if c < image_center)
                    right_side_count = sum(1 for c in centroids if c >= image_center)
                    
                    # 如果所有质心都在一侧，判定偏向该侧
                    if left_side_count > 0 and right_side_count == 0:
                        print(f"[黄线检测-YCY] 所有质心都在左侧({left_side_count}个)，判定偏向左侧")
                        return 'left'
                    elif right_side_count > 0 and left_side_count == 0:
                        print(f"[黄线检测-YCY] 所有质心都在右侧({right_side_count}个)，判定偏向右侧")
                        return 'right'
                
                # 计算图像中心和阈值
                image_center = roi_width / 2
                threshold = roi_width * position_threshold  # 使用可配置的位置阈值
                
                if len(centroids) >= 2:
                    # 两个或更多质心：找到最左和最右的质心
                    left_centroid = min(centroids)
                    right_centroid = max(centroids)
                    lines_center = (left_centroid + right_centroid) / 2
                    
                    print(f"[黄线检测-YCY] 左质心: {left_centroid}, 右质心: {right_centroid}, 黄线中心: {lines_center}")
                    
                    # 判断机器人位置
                    if safe_compare(image_center, lines_center - threshold, '<'):
                        print(f"[黄线检测-YCY] 位置判断: LEFT (偏移: {safe_float(lines_center) - safe_float(image_center):.1f})")
                        return 'left'
                    elif safe_compare(image_center, lines_center + threshold, '>'):
                        print(f"[黄线检测-YCY] 位置判断: RIGHT (偏移: {safe_float(image_center) - safe_float(lines_center):.1f})")
                        return 'right'
                    else:
                        print(f"[黄线检测-YCY] 位置判断: CENTER (偏移: {abs(safe_float(image_center) - safe_float(lines_center)):.1f})")
                        return 'center'
                        
                elif len(centroids) == 1:
                    # 只检测到一个轮廓
                    centroid = centroids[0]
                    print(f"[黄线检测-YCY] 只检测到一个轮廓质心: {centroid}, 图像中心: {image_center}")
                    
                    if safe_compare(centroid, image_center, '<'):
                        return 'left'
                    else:
                        return 'right'
            
            print("[黄线检测-YCY] 未能检测到足够的特征，默认返回center")
            return 'center'
            
        except Exception as e:
            print(f"[黄线检测-YCY] 检测出错: {str(e)}")
            return 'center'
    
    def detect_distance_to_yellow(self, image: np.ndarray, target_position: str = 'center', 
                                roi_params: dict = None, camera_type: str = 'rgb', 
                                threshold_params: dict = None) -> str:
        """
        距离黄线位置检测(dy)：输入图片，输出前，后，中
        计算与中间黄线的距离
        
        Args:
            image: 输入图像 (BGR格式)
            target_position: 目标位置 ('front', 'back', 'center')
            roi_params: ROI区域参数
            camera_type: 相机类型 ('rgb', 'fisheye_left', 'fisheye_right')
            threshold_params: 阈值参数，如 {'distance_threshold': 0.15, 'area_threshold': 300}
            
        Returns:
            位置字符串: 'front' (需要前进), 'back' (需要后退), 'center' (位置正确)
        """
        if image is None:
            return 'center'
        
        # 获取阈值参数
        distance_threshold = threshold_params.get('distance_threshold', 0.15) if threshold_params else 0.15
        area_threshold = threshold_params.get('area_threshold', 300) if threshold_params else 300
        
        try:
            # 获取图像尺寸
            height, width = image.shape[:2]
            
            # 应用ROI参数，专门用于距离检测
            if roi_params:
                roi_top = int(height * roi_params.get('top_ratio', 0.6))
                roi_bottom = int(height * roi_params.get('bottom_ratio', 1.0))
                roi_left = int(width * roi_params.get('left_ratio', 0.0))
                roi_right = int(width * roi_params.get('right_ratio', 1.0))
                roi_image = image[roi_top:roi_bottom, roi_left:roi_right]
                roi_height = roi_bottom - roi_top
            else:
                # 使用中间区域进行距离检测
                roi_top = int(height * 0.3)
                roi_bottom = int(height * 0.7)
                roi_image = image[roi_top:roi_bottom, :]
                roi_height = roi_bottom - roi_top
            
            print(f"[黄线检测-DY] 处理{camera_type}相机图像，目标位置: {target_position}，ROI区域: {roi_image.shape}")
            print(f"[黄线检测-DY] 使用阈值 - 距离阈值: {distance_threshold}, 面积阈值: {area_threshold}")
            
            # 转换为HSV并检测黄色
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
            mask = mask.astype(np.uint8)  # 确保是 8 位单通道

            # 检测轮廓并计算质心
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                 _, contours, hierarchy = contours_result
            else:
                contours, hierarchy = contours_result

            valid_contours = []
            for c in contours:
                if c is not None and len(c) > 0 and c.dtype in [np.int32, np.float32]:
                    valid_contours.append(c)
            print(f"[黄线检测-YCY] 检测到valid_contours {len(valid_contours)} 个轮廓")

            if valid_contours:
                contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)
          
                # 过滤符合面积阈值的轮廓，然后找到质心x最接近中点的轮廓
                valid_contours = []
                roi_width = roi_image.shape[1]
                roi_center_x = roi_width / 2
                
                for contour in contours:
                    if cv2.contourArea(contour) > area_threshold:
                        M = cv2.moments(contour)
                        if safe_float(M["m00"]) != 0:
                            cx = float(M["m10"] / M["m00"])  # X坐标（水平位置）
                            cy = float(M["m01"] / M["m00"])  # Y坐标（垂直位置）
                            x_distance = abs(cx - roi_center_x)  # 到中心的距离
                            valid_contours.append((contour, cx, cy, x_distance))
                
                if valid_contours:
                    # 选择质心x最接近中点的轮廓
                    selected_contour, cx, cy, x_distance = min(valid_contours, key=lambda x: x[3])
                    print(f"[黄线检测-DY] 找到 {len(valid_contours)} 个有效轮廓，选择质心x最接近中点的轮廓")
                    print(f"[黄线检测-DY] 选择的轮廓质心: X={cx}, Y={cy}, 到中心距离={x_distance}")
                    
                    # 使用选择的轮廓的Y坐标进行距离判断
                    cy = cy  # 使用已计算的Y坐标
                        
                    # 计算相对于ROI中心的位置
                    roi_center_y = roi_height / 2
                    threshold = roi_height * distance_threshold  # 使用可配置的距离阈值
                    
                    print(f"[黄线检测-DY] 黄线质心Y: {cy}, ROI中心Y: {roi_center_y}, 阈值: {threshold}")
                    
                    # 判断距离位置
                    if safe_compare(cy, roi_center_y - threshold, '<'):
                        print("[黄线检测-DY] 黄线在前方，需要前进")
                        return 'front'
                    elif safe_compare(cy, roi_center_y + threshold, '>'):
                        print("[黄线检测-DY] 黄线在后方，需要后退") 
                        return 'back'
                    else:
                        print("[黄线检测-DY] 距离适中，位置正确")
                        return 'center'
            
            print("[黄线检测-DY] 未检测到黄线，默认返回center")
            return 'center'
            
        except Exception as e:
            print(f"[黄线检测-DY] 检测出错: {str(e)}")
            return 'center'
    
    def detect_distance_to_yellow_walk(self, image: np.ndarray, target_position: str = 'center', 
                                     roi_params: dict = None, camera_type: str = 'rgb', 
                                     threshold_params: dict = None) -> str:
        """
        针对yellow_line_walk的距离检测：输入图片，输出前，后，中
        1. 先判断黄色区域的最底部与底边的距离，如果超过某个阈值则返回向后
        2. 然后判断，如果质心x距离中点的距离，超过某个阈值时，则返回向前
        
        Args:
            image: 输入图像 (BGR格式)
            target_position: 目标位置 ('front', 'back', 'center')
            roi_params: ROI区域参数
            camera_type: 相机类型 ('rgb', 'fisheye_left', 'fisheye_right')
            threshold_params: 阈值参数，如 {'distance_threshold': 0.15, 'area_threshold': 300, 
                                          'bottom_distance_threshold': 50, 'x_center_threshold': 30}
            
        Returns:
            位置字符串: 'front' (需要前进), 'back' (需要后退), 'center' (位置正确)
        """
        if image is None:
            return 'center'
        
        # 获取阈值参数
        distance_threshold = threshold_params.get('distance_threshold', 0.15) if threshold_params else 0.15
        area_threshold = threshold_params.get('area_threshold', 300) if threshold_params else 300
        bottom_distance_threshold = threshold_params.get('bottom_distance_threshold', 50) if threshold_params else 50
        x_center_threshold = threshold_params.get('x_center_threshold', 30) if threshold_params else 30
        
        try:
            # 获取图像尺寸
            height, width = image.shape[:2]
            
            # 应用ROI参数，专门用于距离检测
            if roi_params:
                roi_top = int(height * roi_params.get('top_ratio', 0.6))
                roi_bottom = int(height * roi_params.get('bottom_ratio', 1.0))
                roi_left = int(width * roi_params.get('left_ratio', 0.0))
                roi_right = int(width * roi_params.get('right_ratio', 1.0))
                roi_image = image[roi_top:roi_bottom, roi_left:roi_right]
                roi_height = roi_bottom - roi_top
            else:
                # 使用中间区域进行距离检测
                roi_top = int(height * 0.3)
                roi_bottom = int(height * 0.7)
                roi_image = image[roi_top:roi_bottom, :]
                roi_height = roi_bottom - roi_top
            
            print(f"[黄线检测-WALK] 处理{camera_type}相机图像，目标位置: {target_position}，ROI区域: {roi_image.shape}")
            print(f"[黄线检测-WALK] 使用阈值 - 距离阈值: {distance_threshold}, 面积阈值: {area_threshold}")
            print(f"[黄线检测-WALK] 底部距离阈值: {bottom_distance_threshold}, X中心阈值: {x_center_threshold}")
            
            # 转换为HSV并检测黄色
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
             
            # 基于轮廓的质心检测
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                 _, contours, hierarchy = contours_result
            else:
                contours, hierarchy = contours_result
            print(f"[黄线检测-YCY] 检测到contours {len(contours)} 个轮廓")
            valid_contours = []
            for c in contours:
                if c is not None and len(c) > 0 and c.dtype in [np.int32, np.float32]:
                    valid_contours.append(c)
            print(f"[黄线检测-YCY] 检测到valid_contours {len(valid_contours)} 个轮廓")
            if valid_contours:
                contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)
                # 过滤符合面积阈值的轮廓，然后找到质心x最接近中点的轮廓
            
                valid_contours = []
                roi_width = roi_image.shape[1]
                roi_center_x = roi_width / 2
                
                for contour in contours:
                    try:
                        # 安全计算面积
                        area = cv2.contourArea(contour)
                        if area > area_threshold:
                            M = cv2.moments(contour)
                            if safe_float(M["m00"]) != 0:
                                cx = float(M["m10"] / M["m00"])  # X坐标（水平位置）
                                cy = float(M["m01"] / M["m00"])  # Y坐标（垂直位置）
                                x_distance = abs(cx - roi_center_x)  # 到中心的距离
                                
                                # 计算轮廓的最底部位置
                                bottom_y = np.max(contour[:, :, 1])  # 轮廓中Y坐标的最大值
                                valid_contours.append((contour, cx, cy, x_distance, bottom_y))
                    except Exception as e:
                        print(f"[黄线检测-WALK] 轮廓面积计算失败: {str(e)}")
                        continue
                
                if valid_contours:
                    # 选择质心x最接近中点的轮廓
                    selected_contour, cx, cy, x_distance, bottom_y = min(valid_contours, key=lambda x: x[3])
                    print(f"[黄线检测-WALK] 找到 {len(valid_contours)} 个有效轮廓，选择质心x最接近中点的轮廓")
                    print(f"[黄线检测-WALK] 选择的轮廓质心: X={cx}, Y={cy}, 到中心距离={x_distance}")
                    print(f"[黄线检测-WALK] 轮廓最底部Y: {bottom_y}, ROI高度: {roi_height}")
                    
                    # 1. 先判断黄色区域的最底部与底边的距离
                    bottom_distance = roi_height - bottom_y  # 最底部到ROI底边的距离
                    print(f"[黄线检测-WALK] 底部距离: {bottom_distance}, 阈值: {bottom_distance_threshold}")
                    
                    if bottom_distance < bottom_distance_threshold:
                        print("[黄线检测-WALK] 黄色区域距离底边太近，需要后退")
                        return 'back'
                    
                    # 2. 然后判断质心x距离中点的距离
                    print(f"[黄线检测-WALK] 质心X距离中点: {x_distance}, 阈值: {x_center_threshold}")
                    
                    if x_distance > x_center_threshold:
                        print("[黄线检测-WALK] 质心X距离中点太远，需要前进")
                        return 'front'
    
            return 'center'
            
        except Exception as e:
            print(f"[黄线检测-WALK] 检测出错: {str(e)}")
            return 'center'
    
    def detect_position(self, image: np.ndarray, detection_type: str = 'ycy', 
                       roi_params: dict = None, camera_type: str = 'rgb', 
                       target_position: str = 'center', threshold_params: dict = None) -> str:
        """
        统一的检测接口，支持两种检测类型
        
        Args:
            image: 输入图像 (BGR格式)
            detection_type: 检测类型 ('ycy' - 两条黄线中间, 'dy' - 距离黄线位置)
            roi_params: ROI区域参数，如 {'top_ratio': 0.6, 'bottom_ratio': 1.0, 'left_ratio': 0.0, 'right_ratio': 1.0}
            camera_type: 相机类型 ('rgb', 'fisheye_left', 'fisheye_right')
            target_position: 目标位置（仅用于dy类型）
            threshold_params: 阈值参数，如 {'position_threshold': 0.05, 'area_threshold': 500, 'distance_threshold': 0.15}
            
        Returns:
            位置字符串：
            - ycy类型: 'left', 'right', 'center' 
            - dy类型: 'front', 'back', 'center'
        """
        # 更新阈值参数
        if threshold_params:
            original_position_threshold = self.position_threshold
            if 'position_threshold' in threshold_params:
                self.position_threshold = threshold_params['position_threshold']
        
        try:
            if detection_type == 'ycy':
                result = self.detect_yellow_between_center(image, roi_params, camera_type, threshold_params)
            elif detection_type == 'dy':
                result = self.detect_distance_to_yellow(image, target_position, roi_params, camera_type, threshold_params)
            else:
                # 默认使用ycy类型（向后兼容）
                result = self.detect_yellow_between_center(image, roi_params, camera_type, threshold_params)
            
            # 恢复原始阈值参数
            if threshold_params and 'position_threshold' in threshold_params:
                self.position_threshold = original_position_threshold
                
            return result
            
        except Exception as e:
            # 恢复原始阈值参数
            if threshold_params and 'position_threshold' in threshold_params:
                self.position_threshold = original_position_threshold
            print(f"[黄线检测] 统一接口检测出错: {str(e)}")
            return 'center'

    
    def visualize_detection(self, image: np.ndarray, save_path: Optional[str] = None) -> np.ndarray:
        """
        可视化检测结果 - 针对鱼眼相机优化
        
        Args:
            image: 输入图像 (BGR格式)
            save_path: 保存路径（可选）
            
        Returns:
            带有检测结果的图像
        """
        if image is None:
            return None
        
        try:
            # 复制图像用于绘制
            result_image = image.copy()
            height, width = image.shape[:2]
            
            # 绘制ROI区域边界
            roi_start = int(height * (1 - self.roi_bottom_ratio))
            cv2.line(result_image, (0, roi_start), (width, roi_start), (255, 255, 0), 2)
            cv2.putText(result_image, "ROI", (10, roi_start - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # 提取ROI区域进行检测
            roi_image = image[roi_start:, :]
            
            # 转换为HSV并创建掩码
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
            
            # 检测轮廓
            contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_result) == 3:
                 _, contours, hierarchy = contours_result
            else:
                contours, hierarchy = contours_result
            
            if contours:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                
                # 绘制轮廓和质心
                for i, contour in enumerate(contours[:5]):
                    if cv2.contourArea(contour) > 500:
                        # 调整轮廓坐标到原图
                        contour_adjusted = contour.copy()
                        contour_adjusted[:, :, 1] += roi_start
                        
                        # 绘制轮廓
                        cv2.drawContours(result_image, [contour_adjusted], -1, (0, 255, 0), 2)
                        
                        # 计算和绘制质心
                        M = cv2.moments(contour)
                        if safe_float(M["m00"]) != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"]) + roi_start
                            cv2.circle(result_image, (cx, cy), 8, (0, 0, 255), -1)
                            cv2.putText(result_image, f"C{i+1}", (cx + 15, cy), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 绘制图像中心线
            image_center = width // 2
            cv2.line(result_image, (image_center, 0), (image_center, height), (255, 0, 0), 2)
            cv2.putText(result_image, "Image Center", (image_center + 10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # 检测位置并添加文本
            position = self.detect_position(image)
            cv2.putText(result_image, f"Position: {position}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 添加检测参数信息
            param_text = [
                f"HSV: {self.lower_yellow} - {self.upper_yellow}",
                f"Threshold: {self.position_threshold:.2f}",
                f"ROI: {self.roi_bottom_ratio:.1f}"
            ]
            
            for i, text in enumerate(param_text):
                cv2.putText(result_image, text, (10, 100 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 保存图像（如果指定了路径）
            if save_path:
                cv2.imwrite(save_path, result_image)
                # 同时保存掩码图像用于调试
                mask_full = np.zeros((height, width), dtype=np.uint8)
                mask_full[roi_start:, :] = mask
                cv2.imwrite(save_path.replace('.jpg', '_mask.jpg'), mask_full)
            
            return result_image
            
        except Exception as e:
            print(f"[黄线检测] 可视化过程中出错: {str(e)}")
            return image





if __name__ == "__main__":
    """测试代码"""
    import os
    
    # 创建检测器实例
    detector = YellowLineDetector()
    
    # 测试用例（如果有测试图像的话）
    test_image_path = "test_image.jpg"
    if os.path.exists(test_image_path):
        # 读取测试图像
        image = cv2.imread(test_image_path)
        
        if image is not None:
            # 检测位置
            position = detector.detect_position(image)
            print(f"检测到的位置: {position}")
            
            # 可视化结果
            result = detector.visualize_detection(image, "detection_result.jpg")
            print("检测结果已保存到 detection_result.jpg")
        else:
            print("无法读取测试图像")
    else:
        print("未找到测试图像，跳过测试")
        print("黄线检测器模块已准备就绪")