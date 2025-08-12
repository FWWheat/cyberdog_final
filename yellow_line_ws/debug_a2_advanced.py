#!/usr/bin/env python3

import cv2
import numpy as np
import pytesseract
import sys

def preprocess_for_large_text(image):
   
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # 多种预处理方法
    methods = {}
    
    # 方法1: 简单灰度
    methods['gray'] = gray
    

    return methods

def detect_code_in_text(text):
    """从OCR文本中检测A-1/A-2/B-1/B-2"""
    if not text:
        return None
    
    text_upper = text.upper()
    
    # 精确匹配
    expected_codes = ['A-1', 'A-2', 'B-1', 'B-2']
    for code in expected_codes:
        if code in text_upper:
            return code
    
    # 模糊匹配
    # 检查A-X模式
    has_a_like = any(char in text_upper for char in ['A', 'Α'])
    has_1_like = any(char in text for char in ['1'])
    has_2_like = any(char in text for char in ['2'])
    
    if has_a_like:
        if has_2_like or '2' in text:
            return 'A-2'
        elif has_1_like or '1' in text:
            return 'A-1'
    
    # 检查B-X模式
    has_b_like = any(char in text_upper for char in ['B', 'Β'])
    
    if has_b_like:
        if has_2_like or '2' in text:
            return 'B-2'
        elif has_1_like or '1' in text:
            return 'B-1'
    
    return None

def detect_code_in_image(image_path):
    """检测图像中的代码"""
    image = cv2.imread(image_path)
    if image is None:
        return None
            # 先划定感兴趣区域(ROI) - 取图像顶部30%，中间25%部分
    h, w = image.shape[:2]
    
    # 垂直方向：取顶部30%
    roi_y1 = int(h * 0.1)
    roi_y2 = int(h * 0.25)
    
    # 水平方向：取中间25%（左右各37.5%边距）
    roi_margin_w = int(w * 0.4)  # 水平边距37.5%
    roi_x1 = roi_margin_w
    roi_x2 = w - roi_margin_w
    
    # 提取ROI区域
    roi_image = image[roi_y1:roi_y2, roi_x1:roi_x2]
    
    # 获取所有预处理版本
    processed_images = preprocess_for_large_text(roi_image)
        
    
    # OCR配置
    configs = [
        '--oem 3 --psm 6',# 单个文本块
    ]
    
    # 测试所有组合
    for process_name, processed_img in processed_images.items():
        for config in configs:
            try:
                # 使用image_to_string获取完整文本
                text = pytesseract.image_to_string(processed_img, config=config).strip()
                print(text)
                detected_code = detect_code_in_text(text)
                if detected_code:
                    return detected_code

            except Exception:
                continue
    
    return None

if __name__ == "__main__":
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        image_path = input("Enter image path: ").strip()
    
    if not image_path:
        print("没有")
        sys.exit(1)
    
    result = detect_code_in_image(image_path)
    if result:
        print(result)
    else:
        print("没有")