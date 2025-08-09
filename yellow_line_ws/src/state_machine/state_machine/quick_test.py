#!/usr/bin/env python3

"""
简单的黄线检测测试脚本
使用方法: python3 quick_test.py <图像路径>
"""

import cv2
import sys
import os
from yellow_line_detector import YellowLineDetector

def test_image(image_path):
    """测试单张图像"""
    
    print(f"测试图像: {image_path}")
    
    # 检查文件
    if not os.path.exists(image_path):
        print("❌ 文件不存在")
        return
    
    # 读取图像
    image = cv2.imread(image_path)
    if image is None:
        print("❌ 无法读取图像")
        return
    
    print(f"图像尺寸: {image.shape[1]} x {image.shape[0]}")
    
    # 创建检测器（统一配置）
    detector = YellowLineDetector()
    print("使用统一的检测器配置")
    
    # 执行检测
    print("\n--- 检测过程 ---")
    position = detector.detect_position(image)
    
    # 输出结果
    print(f"\n🎯 最终检测结果: {position.upper()}")
    
    # 给出位置调整建议
    print("\n📋 位置调整建议:")
    if position == 'left':
        print("   机器人靠近左边线")
        print("   鱼眼相机模式: 建议向后走")
        print("   RGB相机模式: 建议向右转")
    elif position == 'right':
        print("   机器人靠近右边线")
        print("   鱼眼相机模式: 建议向前走")
        print("   RGB相机模式: 建议向左转")
    else:
        print("   机器人位置正确 → 在两条黄线中间")
    
    # 生成可视化
    base_name = os.path.splitext(os.path.basename(image_path))[0]
    result_path = f"{base_name}_result.jpg"
    
    detector.visualize_detection(image, result_path)
    print(f"✅ 结果图像: {result_path}")
    print(f"✅ 掩码图像: {result_path.replace('.jpg', '_mask.jpg')}")

def main():
    if len(sys.argv) != 2:
        print("使用方法:")
        print(f"  python3 {sys.argv[0]} <图像路径>")
        print(f"\n示例:")
        print(f"  python3 {sys.argv[0]} image1.jpg")
        print(f"  python3 {sys.argv[0]} fisheye_image.jpg")
        return
    
    image_path = sys.argv[1]
    
    print("=" * 50)
    print("黄线检测快速测试")
    print("=" * 50)
    
    try:
        test_image(image_path)
        print(f"\n测试完成!")
        
    except Exception as e:
        print(f"错误: 测试过程中出现异常")
        print(f"错误信息: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()