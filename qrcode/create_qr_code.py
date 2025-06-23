"""
用法：

    create_qr_code.py "一些文本" myqrcode.jpg

"""
from __future__ import print_function
import sys
import qrcode

if __name__ == '__main__':
    # 从命令行参数获取数据和文件名
    data = sys.argv[1]  # QR 码中包含的文本
    fn = sys.argv[2]    # 保存生成的 QR 码图像的文件名
    img = qrcode.make(data)  # 生成 QR 码图像
    save_path = f'../img/{fn}'  # 定义保存路径
    img.save(save_path, 'JPEG')  # 将图像保存为 JPEG 格式
    print('已写入图像:', fn, file=sys.stderr)  # 输出保存的文件名
