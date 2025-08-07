#!/usr/bin/env python3

"""
黄线检测测试脚本
用于测试黄线检测算法在真实图像上的效果
"""

import cv2
import numpy as np
import os
from yellow_line_detector import YellowLineDetector

def test_single_image(image_path, expected_position=None):
    """测试单张图像"""
    print(f"\n{'='*50}")
    print(f"测试图像: {image_path}")
    print(f"期望位置: {expected_position if expected_position else '未知'}")
    print('='*50)
    
    # 读取图像
    if not os.path.exists(image_path):
        print(f"错误: 图像文件不存在 - {image_path}")
        return False
    
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法读取图像 - {image_path}")
        return False
    
    print(f"图像尺寸: {image.shape[1]}x{image.shape[0]}")
    
    # 创建检测器
    detector = YellowLineDetector()
    
    # 执行检测
    detected_position = detector.detect_position(image)
    
    # 输出结果
    print(f"\n检测结果: {detected_position}")
    if expected_position:
        is_correct = detected_position == expected_position
        print(f"期望结果: {expected_position}")
        print(f"检测正确: {'✓' if is_correct else '✗'}")
    
    # 生成可视化结果
    base_name = os.path.splitext(os.path.basename(image_path))[0]
    output_path = f"/tmp/{base_name}_detection_result.jpg"
    
    result_image = detector.visualize_detection(image, output_path)
    if result_image is not None:
        print(f"可视化结果已保存到: {output_path}")
        print(f"掩码图像已保存到: {output_path.replace('.jpg', '_mask.jpg')}")
    
    return expected_position is None or detected_position == expected_position

def test_with_different_parameters():
    """测试不同的检测参数"""
    print(f"\n{'='*50}")
    print("测试不同检测参数")
    print('='*50)
    
    # 参数组合
    test_params = [
        {"name": "宽松参数", "lower_yellow": (10, 60, 60), "upper_yellow": (40, 255, 255), 
         "position_threshold": 0.2, "roi_bottom_ratio": 0.7},
        {"name": "严格参数", "lower_yellow": (20, 100, 100), "upper_yellow": (30, 255, 255), 
         "position_threshold": 0.1, "roi_bottom_ratio": 0.5},
        {"name": "默认参数", "lower_yellow": (15, 80, 80), "upper_yellow": (35, 255, 255), 
         "position_threshold": 0.15, "roi_bottom_ratio": 0.6},
    ]
    
    # 如果有测试图像，使用第一张进行参数测试
    test_image_path = "/tmp/test_image.jpg"
    if os.path.exists(test_image_path):
        image = cv2.imread(test_image_path)
        
        for params in test_params:
            print(f"\n--- {params['name']} ---")
            detector = YellowLineDetector(
                lower_yellow=params["lower_yellow"],
                upper_yellow=params["upper_yellow"], 
                position_threshold=params["position_threshold"],
                roi_bottom_ratio=params["roi_bottom_ratio"]
            )
            
            position = detector.detect_position(image)
            print(f"检测结果: {position}")
            
            # 保存可视化结果
            output_path = f"/tmp/test_params_{params['name']}.jpg"
            detector.visualize_detection(image, output_path)
            print(f"结果已保存: {output_path}")

def create_test_images_from_user_data():
    """创建用户提供的测试图像（示例代码）"""
    print(f"\n{'='*50}")
    print("创建测试图像")
    print('='*50)
    
    # 这里应该是用户提供的图像数据
    # 由于无法直接访问用户上传的图像，我们创建一些提示
    
    test_cases = [
        {"name": "image1_right", "expected": "right", "description": "机器人位置靠近右边线"},
        {"name": "image2_right", "expected": "right", "description": "机器人位置靠近右边线"}, 
        {"name": "image3_center", "expected": "center", "description": "机器人位置在中间"},
        {"name": "image4_left", "expected": "left", "description": "机器人位置靠近左边线"},
    ]
    
    print("请将测试图像保存到以下路径:")
    for case in test_cases:
        image_path = f"/tmp/{case['name']}.jpg"
        print(f"  {image_path} - {case['description']}")
    
    return test_cases

def main():
    """主测试函数"""
    print("黄线检测算法测试")
    print("="*50)
    
    # 创建测试图像信息
    test_cases = create_test_images_from_user_data()
    
    # 测试每张图像
    correct_count = 0
    total_count = 0
    
    for case in test_cases:
        image_path = f"/tmp/{case['name']}.jpg"
        
        if os.path.exists(image_path):
            total_count += 1
            is_correct = test_single_image(image_path, case['expected'])
            if is_correct:
                correct_count += 1
        else:
            print(f"\n图像不存在，跳过测试: {image_path}")
    
    # 测试不同参数（如果有图像的话）
    if total_count > 0:
        test_with_different_parameters()
    
    # 输出总结
    print(f"\n{'='*50}")
    print("测试总结")
    print('='*50)
    if total_count > 0:
        accuracy = correct_count / total_count * 100
        print(f"测试图像数量: {total_count}")
        print(f"检测正确数量: {correct_count}")
        print(f"检测准确率: {accuracy:.1f}%")
    else:
        print("未找到测试图像，请将图像保存到 /tmp/ 目录下")
        print("图像命名格式:")
        print("  image1_right.jpg  - 预期结果: right")
        print("  image2_right.jpg  - 预期结果: right") 
        print("  image3_center.jpg - 预期结果: center")
        print("  image4_left.jpg   - 预期结果: left")
    
    print(f"\n使用说明:")
    print(f"1. 将测试图像保存到 /tmp/ 目录")
    print(f"2. 运行: python3 test_yellow_line_detection.py")
    print(f"3. 查看 /tmp/ 目录下的检测结果图像")

if __name__ == "__main__":
    main()