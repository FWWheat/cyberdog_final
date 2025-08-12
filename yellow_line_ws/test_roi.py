#!/usr/bin/env python3

from re import I
import cv2
import numpy as np

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
    roi_y1 = int(h * 0.1)
    roi_y2 = int(h * 0.25)
    
    # 水平方向：取中间25%（左右各37.5%边距）
    roi_margin_w = int(w * 0.4)  # 水平边距37.5%
    roi_x1 = roi_margin_w
    roi_x2 = w - roi_margin_w
    
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
    # 测试图像
    test_images = ["1.png", "2.png", "3.png", "4.png", "5.png", "6.png"]
    
    for img_name in test_images:
        try:
            print(f"\n处理图像: {img_name}")
            output_name = f"test_results/roi_{img_name}"
            input_name = f"img3/{img_name}"
            print(input_name)
            draw_roi(input_name, output_name)
        except Exception as e:
            print(f"处理 {img_name} 时出错: {e}")
            continue
    
    print("\n所有图像处理完成！")