#!/usr/bin/env python3
"""
颜色分析工具 - 分析图像中的颜色分布，特别是红色区域
用于调试红色限高杆检测的颜色范围设置
"""

import cv2
import numpy as np
import sys
import os

def analyze_image_colors(image_path):
    """分析图像颜色分布"""
    if not os.path.exists(image_path):
        print(f"图片不存在: {image_path}")
        return
    
    image = cv2.imread(image_path)
    if image is None:
        print(f"无法读取图片: {image_path}")
        return
    
    print(f"分析图片: {image_path}")
    print(f"图片尺寸: {image.shape}")
    
    # 转换为HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 创建显示图像
    height, width = image.shape[:2]
    display_image = np.zeros((height, width * 3, 3), dtype=np.uint8)
    display_image[:, :width] = image  # 原图
    
    # 分析红色区域 - 用不同颜色标记不同范围
    red_ranges = [
        # 名称，下限，上限，显示颜色
        ("Red1", [0, 20, 20], [20, 255, 255], (0, 0, 255)),
        ("Red2", [140, 20, 20], [180, 255, 255], (0, 100, 255)), 
        ("Orange", [5, 50, 80], [25, 255, 255], (0, 165, 255)),
        ("Dark", [160, 30, 30], [180, 255, 180], (0, 0, 128)),
        ("Wide", [0, 10, 10], [30, 255, 255], (255, 0, 255)),  # 极宽红色范围
    ]
    
    combined_mask = np.zeros((height, width), dtype=np.uint8)
    
    for i, (name, lower, upper, color) in enumerate(red_ranges):
        lower_np = np.array(lower)
        upper_np = np.array(upper)
        mask = cv2.inRange(hsv, lower_np, upper_np)
        
        # 计算覆盖面积
        coverage = np.sum(mask > 0) / (height * width) * 100
        print(f"{name}: HSV[{lower[0]}-{upper[0]}, {lower[1]}-{upper[1]}, {lower[2]}-{upper[2]}] 覆盖: {coverage:.2f}%")
        
        # 在显示图像上标记
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = color
        display_image[:, width:width*2] = cv2.addWeighted(display_image[:, width:width*2], 0.7, colored_mask, 0.3, 0)
        
        combined_mask = cv2.bitwise_or(combined_mask, mask)
    
    # 显示组合掩码
    display_image[:, width*2:width*3, 0] = combined_mask
    display_image[:, width*2:width*3, 1] = combined_mask  
    display_image[:, width*2:width*3, 2] = combined_mask
    
    # 分析HSV直方图
    print("\n=== HSV直方图分析 ===")
    h_hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])
    s_hist = cv2.calcHist([hsv], [1], None, [256], [0, 256])
    v_hist = cv2.calcHist([hsv], [2], None, [256], [0, 256])
    
    # 找到色相的主要峰值
    h_peaks = []
    for i in range(len(h_hist)):
        if h_hist[i] > height * width * 0.001:  # 超过0.1%的像素
            h_peaks.append((i, h_hist[i][0]))
    
    h_peaks.sort(key=lambda x: x[1], reverse=True)
    print("主要色相值(前10):")
    for i, (hue, count) in enumerate(h_peaks[:10]):
        percentage = count / (height * width) * 100
        print(f"  H={hue:3d}: {percentage:5.2f}% ({int(count):6d} 像素)")
    
    # 点击分析功能
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if x < width:  # 点击原图
                bgr = image[y, x]
                hsv_val = hsv[y, x]
                print(f"点击位置({x}, {y}): BGR={bgr} HSV={hsv_val}")
    
    cv2.namedWindow('Color Analysis')
    cv2.setMouseCallback('Color Analysis', mouse_callback)
    
    # 添加文字说明
    cv2.putText(display_image, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(display_image, "Red Detection", (width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(display_image, "Combined Mask", (width*2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(display_image, "Click on original image to analyze pixel", (10, height - 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 显示结果
    cv2.imshow('Color Analysis', display_image)
    
    # 保存分析结果
    analysis_path = image_path.replace('.png', '_color_analysis.png')
    cv2.imwrite(analysis_path, display_image)
    print(f"\n颜色分析结果已保存: {analysis_path}")
    
    print("\n点击原图可查看像素颜色值，按任意键退出...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        print("用法: python3 analyze_color.py <图片路径>")
        print("例如: python3 analyze_color.py img3/14.png")
        return
    
    image_path = sys.argv[1]
    analyze_image_colors(image_path)

if __name__ == '__main__':
    main()