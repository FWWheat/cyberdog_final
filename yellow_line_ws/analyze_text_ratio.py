#!/usr/bin/env python3
import cv2
import numpy as np
import os

def calculate_text_ratio(image_path):
    """计算文字在图片中的占比"""
    image = cv2.imread(image_path)
    if image is None:
        return None
    
    h, w = image.shape[:2]
    print(f"\n分析图片: {os.path.basename(image_path)}")
    print(f"图片尺寸: {w}x{h}")
    
    # 找到A-2标识的位置
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # 使用模板匹配或者简单的文字区域检测
    # 这里我们手动分析标识位置
    
    # 根据观察，A-2标识大约在图片的中上部分
    # 估算文字区域
    text_x = int(w * 0.45)  # 文字大约在图片中间偏左
    text_y = int(h * 0.15)  # 文字在图片上部
    text_w = int(w * 0.08)  # 文字宽度约占图片8%
    text_h = int(h * 0.08)  # 文字高度约占图片8%
    
    # 计算文字区域面积
    text_area = text_w * text_h
    total_area = w * h
    text_ratio = (text_area / total_area) * 100
    
    print(f"估算文字区域: {text_w}x{text_h}")
    print(f"文字面积占比: {text_ratio:.2f}%")
    
    # 分析ROI区域（当前代码使用的区域）
    roi_y1 = int(h * 0.1)
    roi_y2 = int(h * 0.25)
    roi_margin_w = int(w * 0.4)
    roi_x1 = roi_margin_w
    roi_x2 = w - roi_margin_w
    
    roi_w = roi_x2 - roi_x1
    roi_h = roi_y2 - roi_y1
    roi_area = roi_w * roi_h
    roi_ratio = (roi_area / total_area) * 100
    
    print(f"ROI区域: {roi_w}x{roi_h} (位置: {roi_x1},{roi_y1} to {roi_x2},{roi_y2})")
    print(f"ROI面积占比: {roi_ratio:.2f}%")
    
    # 计算文字在ROI中的占比
    # 检查文字是否在ROI区域内
    text_in_roi = (text_x >= roi_x1 and text_x + text_w <= roi_x2 and 
                   text_y >= roi_y1 and text_y + text_h <= roi_y2)
    
    if text_in_roi:
        text_in_roi_ratio = (text_area / roi_area) * 100
        print(f"文字在ROI中的占比: {text_in_roi_ratio:.2f}%")
    else:
        print("文字可能不完全在ROI区域内！")
        # 计算重叠部分
        overlap_x1 = max(text_x, roi_x1)
        overlap_y1 = max(text_y, roi_y1)
        overlap_x2 = min(text_x + text_w, roi_x2)
        overlap_y2 = min(text_y + text_h, roi_y2)
        
        if overlap_x2 > overlap_x1 and overlap_y2 > overlap_y1:
            overlap_area = (overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1)
            overlap_ratio = (overlap_area / text_area) * 100
            print(f"文字与ROI重叠比例: {overlap_ratio:.2f}%")
        else:
            print("文字与ROI无重叠")
    
    # 显示ROI区域以便验证
    roi_image = image[roi_y1:roi_y2, roi_x1:roi_x2]
    
    return {
        'image_size': (w, h),
        'text_ratio': text_ratio,
        'roi_ratio': roi_ratio,
        'text_in_roi': text_in_roi,
        'roi_image': roi_image
    }

def main():
    img_dir = "img3"
    
    for i in range(1, 7):
        image_path = os.path.join(img_dir, f"{i}.png")
        if os.path.exists(image_path):
            result = calculate_text_ratio(image_path)
            
            # 保存ROI区域图片以便查看
            if result:
                roi_output_path = f"roi_{i}.png"
                cv2.imwrite(roi_output_path, result['roi_image'])
                print(f"ROI区域保存为: {roi_output_path}")
            
            print("-" * 50)

if __name__ == "__main__":
    main()