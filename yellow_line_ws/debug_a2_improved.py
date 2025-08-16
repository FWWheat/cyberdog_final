#!/usr/bin/env python3

import cv2
import numpy as np
import pytesseract
import sys

def preprocess_for_large_text(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    methods = {}
    methods['gray'] = gray
    
    # 添加更多预处理方法提高识别率
    # 二值化
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    methods['binary'] = binary
    
    # 形态学操作去噪
    kernel = np.ones((2,2), np.uint8)
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    methods['cleaned'] = cleaned
    
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
    has_a_like = any(char in text_upper for char in ['A', 'Α'])
    has_1_like = any(char in text for char in ['1','i'])
    has_2_like = any(char in text for char in ['2'])
    
    if has_a_like:
        if has_2_like or '2' in text:
            return 'A-2'
        elif has_1_like or '1' in text:
            return 'A-1'
    
    has_b_like = any(char in text_upper for char in ['B', 'Β','5'])
    
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
    
    h, w = image.shape[:2]
    
    # 改进的ROI设置 - 扩大搜索范围，覆盖更多可能的文字位置
    # 方案1：扩大原始ROI
    roi_configs = [
        # 原始配置（稍作调整）
        {
            'y1': int(h * 0.05),  # 从更上方开始
            'y2': int(h * 0.30),  # 扩大到30%
            'x1': int(w * 0.35),  # 减少左边距
            'x2': int(w * 0.65),  # 减少右边距
            'name': 'extended_roi'
        },
        # 方案2：专门针对文字位置的ROI
        {
            'y1': int(h * 0.08),
            'y2': int(h * 0.25),
            'x1': int(w * 0.40),
            'x2': int(w * 0.60),
            'name': 'focused_roi'
        },
        # 方案3：更大范围的ROI
        {
            'y1': int(h * 0.0),
            'y2': int(h * 0.35),
            'x1': int(w * 0.30),
            'x2': int(w * 0.70),
            'name': 'large_roi'
        },
        # 中间
        {
            'y1': int(h * 0.35),  # 从高度 35% 开始
            'y2': int(h * 0.65),  # 到高度 65%
            'x1': int(w * 0.35),  # 左边裁掉 35%
            'x2': int(w * 0.65),  # 右边裁掉 35%
            'name': 'center_roi'
        },
        # 中间大
        {
            'y1': int(h * 0.3),  # 从高度 35% 开始
            'y2': int(h * 0.6),  # 到高度 65%
            'x1': int(w * 0.3),  # 左边裁掉 35%
            'x2': int(w * 0.7),  # 右边裁掉 35%
            'name': 'center_roi_large'
        }
    ]
    
    # OCR配置 - 添加更多配置选项
    configs = [
        '--oem 3 --psm 6',  # 单个文本块
        '--oem 3 --psm 7',  # 单行文本
        '--oem 3 --psm 8',  # 单个单词
        '--oem 3 --psm 13', # 原始行，不做假设
    ]
    
    # 尝试所有ROI配置
    for roi_config in roi_configs:
        # 提取ROI区域
        roi_image = image[roi_config['y1']:roi_config['y2'], 
                         roi_config['x1']:roi_config['x2']]
        
        if roi_image.size == 0:
            continue
            
        # 获取所有预处理版本
        processed_images = preprocess_for_large_text(roi_image)
        
        # 测试所有组合
        for process_name, processed_img in processed_images.items():
            for config in configs:
                try:
                    text = pytesseract.image_to_string(processed_img, config=config).strip()
                    print(f"ROI: {roi_config['name']}, 预处理: {process_name}, 配置: {config}")
                    print(f"识别文本: '{text}'")
                    
                    detected_code = detect_code_in_text(text)
                    if detected_code:
                        print(f"检测到代码: {detected_code}")
                        return detected_code

                except Exception as e:
                    print(f"OCR错误: {e}")
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
        print(f"最终结果: {result}")
    else:
        print("没有")