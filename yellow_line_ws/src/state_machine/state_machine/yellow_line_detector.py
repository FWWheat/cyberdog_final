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
from typing import Optional, Tuple, List


class YellowLineDetector:
    """黄线检测器类"""
    
    def __init__(self, 
                 lower_yellow: Tuple[int, int, int] = (15, 80, 80),
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
    
    def detect_yellow_lines(self, image: np.ndarray) -> List[float]:
        """
        检测图像中的黄色纵向直线
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            垂直线的x坐标列表
        """
        if image is None:
            return []
        
        try:
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 创建黄色掩码
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 形态学操作去除噪声
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
            
            # 检测边缘
            edges = cv2.Canny(mask, 50, 150, apertureSize=3)
            
            # 霍夫变换检测直线
            lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=self.hough_threshold)
            
            vertical_lines = []
            if lines is not None:
                for line in lines:
                    rho, theta = line[0]
                    # 筛选接近垂直的直线 (角度在80-100度之间或-10到10度之间)
                    angle = np.degrees(theta)
                    if (80 <= angle <= 100) or (-10 <= angle <= 10):
                        # 计算直线的x坐标
                        if np.sin(theta) != 0:
                            x = rho / np.sin(theta)
                            # 确保x坐标在图像范围内
                            if 0 <= x <= image.shape[1]:
                                vertical_lines.append(x)
            
            return vertical_lines
            
        except Exception as e:
            print(f"[黄线检测] 检测过程中出错: {str(e)}")
            return []
    
    def detect_position(self, image: np.ndarray) -> str:
        """
        检测机器人在两条黄色纵向平行线中的位置
        适用于鱼眼相机和RGB相机，使用统一的处理方法
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            位置字符串: 'left' (靠近左线), 'right' (靠近右线), 'center' (在中间)
        """
        if image is None:
            return 'center'
        
        try:
            # 获取图像尺寸
            if len(image.shape) != 3:
                print(f"[黄线检测] 错误：图像不是3通道，shape: {image.shape}")
                return 'center'
            
            height, width = image.shape[:2]
            
            # 提取ROI区域（图像下方部分）
            roi_start = int(height * (1 - self.roi_bottom_ratio))
            roi_image = image[roi_start:, :]
            
            print(f"[黄线检测] 原图尺寸: {width}x{height}, ROI区域: {width}x{roi_image.shape[0]}")
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            
            # 创建黄色掩码
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 形态学操作去除噪声
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
            
            # 基于轮廓的质心检测（统一处理方法）
            contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # 兼容OpenCV不同版本
            if len(contour_result) == 3:
                # OpenCV 3.x: (image, contours, hierarchy)
                _, contours, _ = contour_result
            else:
                # OpenCV 4.x: (contours, hierarchy)
                contours, _ = contour_result
            
            if contours:
                # 找到最大的几个轮廓（假设是黄线）
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                
                # 计算轮廓质心
                centroids = []
                for contour in contours[:5]:  # 取前5个最大轮廓
                    if cv2.contourArea(contour) > 500:  # 面积阈值
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            centroids.append(cx)
                
                print(f"[黄线检测] 检测到 {len(centroids)} 个轮廓质心: {centroids}")
                
                # 计算图像中心
                image_center = width / 2
                threshold = width * self.position_threshold
                
                print(f"[黄线检测] 图像中心: {image_center}, 位置阈值: {self.position_threshold} ({threshold:.1f}像素)")
                
                if len(centroids) >= 2:
                    # 两个或更多质心：找到最左和最右的质心
                    left_centroid = min(centroids)
                    right_centroid = max(centroids)
                    
                    # 计算黄线中心
                    lines_center = (left_centroid + right_centroid) / 2
                    
                    print(f"[黄线检测] 左质心: {left_centroid}, 右质心: {right_centroid}, 黄线中心: {lines_center}")
                    
                    # 判断机器人位置（统一逻辑）
                    if image_center < lines_center - threshold:
                        print(f"[黄线检测] 位置判断: LEFT (偏移: {lines_center - image_center:.1f})")
                        return 'left'
                    elif image_center > lines_center + threshold:
                        print(f"[黄线检测] 位置判断: RIGHT (偏移: {image_center - lines_center:.1f})")
                        return 'right'
                    else:
                        print(f"[黄线检测] 位置判断: CENTER (偏移: {abs(image_center - lines_center):.1f})")
                        return 'center'
                        
                elif len(centroids) == 1:
                    # 只检测到一个轮廓：根据质心位置相对于图像中心判断（统一逻辑）
                    centroid = centroids[0]
                    
                    print(f"[黄线检测] 只检测到一个轮廓质心: {centroid}, 图像中心: {image_center}")
                    
                    if centroid < image_center:
                        print("[黄线检测] 质心在画面左侧，判定机器人偏左")
                        return 'left'
                    else:
                        print("[黄线检测] 质心在画面右侧，判定机器人偏右") 
                        return 'right'
            
            print("[黄线检测] 未能检测到足够的特征，默认返回center")
            return 'center'
            
        except Exception as e:
            print(f"[黄线检测] 位置检测出错: {str(e)}")
            print(f"[黄线检测] 图像信息: shape={image.shape if image is not None else 'None'}")
            import traceback
            print(f"[黄线检测] 错误详情: {traceback.format_exc()}")
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
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
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
                        if M["m00"] != 0:
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


# 便捷函数
def detect_yellow_line_position(image: np.ndarray) -> str:
    """
    便捷函数：检测黄线位置
    
    Args:
        image: 输入图像 (BGR格式)
        
    Returns:
        位置字符串: 'left', 'right', 'center'
    """
    detector = YellowLineDetector()
    return detector.detect_position(image)


def visualize_yellow_line_detection(image: np.ndarray, save_path: Optional[str] = None) -> np.ndarray:
    """
    便捷函数：可视化黄线检测结果
    
    Args:
        image: 输入图像 (BGR格式)
        save_path: 保存路径（可选）
        
    Returns:
        带有检测结果的图像
    """
    detector = YellowLineDetector()
    return detector.visualize_detection(image, save_path)


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