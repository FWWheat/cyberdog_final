#!/usr/bin/env python3

import cv2
import numpy as np
import sys
import os

def draw_roi(image_path, output_path=None):
    """
    在图像上绘制ROI区域
    ROI规格：顶部30%高度，中间25%宽度
    """
    # 读取图像
    image = cv2.imread(image_path)
    if image is None:
        print(f"无法读取图像: {image_path}")
        return
    
    h, w = image.shape[:2]
    print(f"图像尺寸: {w} x {h}")
    
    # 计算ROI区域
    # 垂直方向：取顶部30%
    roi_y1 = int(h * 0.15)
    roi_y2 = int(h * 0.75)
    roi_x1 = int(w * 0.25) 
    roi_x2 = int(w * 0.75) 
    
    print(f"ROI区域: x({roi_x1}-{roi_x2}), y({roi_y1}-{roi_y2})")
    print(f"ROI尺寸: {roi_x2-roi_x1} x {roi_y2-roi_y1}")
    
    # 创建结果图像
    result_image = image.copy()
    
    # 绘制ROI边框（绿色）
    cv2.rectangle(result_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 2)
    
    # 在ROI区域外添加半透明遮罩
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[roi_y1:roi_y2, roi_x1:roi_x2] = 255
    
    # 创建反向遮罩
    mask_inv = cv2.bitwise_not(mask)
    
    # 对非ROI区域应用半透明效果
    overlay = result_image.copy()
    overlay[mask_inv > 0] = overlay[mask_inv > 0] * 0.3  # 30%透明度
    result_image = cv2.addWeighted(result_image, 0.7, overlay, 0.3, 0)
    
    # 添加文本标签
    cv2.putText(result_image, "ROI: Top 30%, Center 25%", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(result_image, f"Size: {roi_x2-roi_x1}x{roi_y2-roi_y1}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 保存或显示结果
    if output_path:
        cv2.imwrite(output_path, result_image)
        print(f"结果已保存到: {output_path}")
    else:
        # 显示图像
        cv2.imshow('ROI Visualization', result_image)
        print("按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    return result_image

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python test_roi.py <image_path> [output_path]")
        print("或者: python test_roi.py <image1> <image2> ... (多个图像)")
        sys.exit(1)
    
    # 如果只有一个图像参数
    if len(sys.argv) == 2:
        image_path = sys.argv[1]
        if not os.path.exists(image_path):
            print(f"图像文件不存在: {image_path}")
            sys.exit(1)
        
        print(f"处理图像: {image_path}")
        draw_roi(image_path)
    
    # 如果有输出路径参数
    elif len(sys.argv) == 3:
        image_path = sys.argv[1]
        output_path = sys.argv[2]
        
        if not os.path.exists(image_path):
            print(f"图像文件不存在: {image_path}")
            sys.exit(1)
        
        print(f"处理图像: {image_path}")
        draw_roi(image_path, output_path)
    
    # 如果有多个图像参数
    else:
        for image_path in sys.argv[1:]:
            if not os.path.exists(image_path):
                print(f"跳过不存在的文件: {image_path}")
                continue
            
            print(f"\n处理图像: {image_path}")
            # 生成输出文件名
            base_name = os.path.splitext(os.path.basename(image_path))[0]
            output_path = f"test_results/roi_{base_name}.png"
            
            # 确保输出目录存在
            os.makedirs("test_results", exist_ok=True)
            
            try:
                draw_roi(image_path, output_path)
            except Exception as e:
                print(f"处理 {image_path} 时出错: {e}")
                continue
        
        print("\n所有图像处理完成！")