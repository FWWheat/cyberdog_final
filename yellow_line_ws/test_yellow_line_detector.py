#!/usr/bin/env python3

"""
黄线检测器测试脚本

用于测试YellowLineDetector的三种检测方法：
1. YCY方法：两条黄线中间检测
2. DY方法：距离黄线位置检测
3. WALK方法：yellow_line_walk距离检测（包含特殊判断逻辑）

使用方法：
    python test_yellow_line_detector.py [image_path]
    
    如果不指定图片路径，会尝试使用默认测试图片
"""

import cv2
import numpy as np
import os
import sys
from typing import Dict, Any

# 添加路径以导入yellow_line_detector模块
sys.path.append('src/state_machine/state_machine')
from yellow_line_detector import YellowLineDetector


def test_single_image(detector, image, image_name="test"):
    """
    测试单张图像的检测功能
    
    Args:
        detector: YellowLineDetector实例
        image: 测试图像
        image_name: 图像名称（用于保存结果）
    
    Returns:
        测试结果字典
    """
    print(f"\n{'='*50}")
    print(f"测试图像: {image_name}")
    print(f"图像尺寸: {image.shape}")
    print(f"{'='*50}")
    
    # 测试参数配置
    roi_params_ycy = {
        'top_ratio': 0.6,
        'bottom_ratio': 1.0,
        'left_ratio': 0.0,
        'right_ratio': 1.0
    }
    
    roi_params_dy = {
        'top_ratio': 0.0,
        'bottom_ratio': 1,
        'left_ratio': 0.0,
        'right_ratio': 1.0
    }
    
    threshold_params_ycy = {
        'position_threshold': 0.08,
        'area_threshold': 800
    }
    
    threshold_params_dy = {
        'distance_threshold': 0.81,
        'area_threshold': 700
    }
    
    threshold_params_walk = {
        'distance_threshold': 0.15,
        'area_threshold': 700,
        'bottom_distance_threshold': 10,
        'x_center_threshold': 80
    }
    
    results = {}
    
    # 测试YCY方法（两条黄线中间检测）
    print("\n--- YCY方法测试 (两条黄线中间检测) ---")
    try:
        ycy_result = detector.detect_position(
            image=image,
            detection_type='ycy',
            roi_params=roi_params_ycy,
            camera_type='fisheye_left',
            threshold_params=threshold_params_ycy
        )
        results['ycy'] = ycy_result
        print(f"YCY检测结果: {ycy_result}")
    except Exception as e:
        print(f"YCY检测出错: {e}")
        results['ycy'] = 'error'
    
    print("\n" + "-"*30)
    
    # 测试DY方法（距离黄线位置检测）
    print("\n--- DY方法测试 (距离黄线位置检测) ---")
    try:
        dy_result = detector.detect_position(
            image=image,
            detection_type='dy',
            roi_params=roi_params_dy,
            camera_type='fisheye_left',
            target_position='center',
            threshold_params=threshold_params_dy
        )
        results['dy'] = dy_result
        print(f"DY检测结果: {dy_result}")
    except Exception as e:
        print(f"DY检测出错: {e}")
        results['dy'] = 'error'
    
    print("\n" + "-"*30)
    
    # 测试WALK方法（yellow_line_walk距离检测）
    print("\n--- WALK方法测试 (yellow_line_walk距离检测) ---")
    try:
        walk_result = detector.detect_distance_to_yellow_walk(
            image=image,
            target_position='center',
            roi_params=roi_params_dy,
            camera_type='fisheye_left',
            threshold_params=threshold_params_walk
        )
        results['walk'] = walk_result
        print(f"WALK检测结果: {walk_result}")
    except Exception as e:
        print(f"WALK检测出错: {e}")
        results['walk'] = 'error'
    
    # 生成可视化结果
    try:
        vis_image = detector.visualize_detection(
            image, 
            save_path=f"test_results/{image_name}_detection_result.jpg"
        )
        print(f"\n可视化结果已保存到: test_results/{image_name}_detection_result.jpg")
        results['visualization'] = 'success'
    except Exception as e:
        print(f"可视化生成失败: {e}")
        results['visualization'] = 'error'
    
    return results



def test_real_image(image_path):
    """测试真实图像"""
    print(f"\n📷 真实图像测试: {image_path}")
    
    if not os.path.exists(image_path):
        print(f"错误：图像文件不存在 - {image_path}")
        return None
    
    # 读取图像
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误：无法读取图像文件 - {image_path}")
        return None
    
    detector = YellowLineDetector()
    
    # 获取图像文件名（不含路径和扩展名）
    image_name = os.path.splitext(os.path.basename(image_path))[0]
    
    # 测试图像
    results = test_single_image(detector, image, f"real_{image_name}")
    
    return results


def main():
    """主函数"""
    print("🚀 黄线检测器综合测试")
    print("="*60)
    
    # 检查命令行参数
    if len(sys.argv) > 1:
        # 如果提供了图像路径，测试真实图像
        image_path = sys.argv[1]
        real_results = test_real_image(image_path)
        if real_results:
            print(f"\n✅ 真实图像测试完成，结果: {real_results}")
    else:
        print("ℹ️  未提供图像路径，将进行合成图像测试")
    

if __name__ == "__main__":
    main()