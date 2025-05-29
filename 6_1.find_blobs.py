# https://longqiu.taobao.com
# @file - By: LQ008 - Fri Oct 11 2024

"""

@file OMV-RT 对色块识别，并输出目标色块的中心坐标位置
有关色块识别 更多参数设置参考：https://docs.openmv.io/library/omv.image.html#image.Image.find_blobs

实验流程：连接好LQ-OMV-RT至电脑
        本例程以红色做简单示例，可以根据示例编写自己的程序
        1、LAB阈值设置，可以在IDE中打开一个 heloWorld 示例工程，从帧缓冲区提取编辑阈值
            然后复制到本程序的target_thresholds列表中
        2、运行程序后，让镜头对准到红色物体，如果身边没有红色，可以自己调整阈值改为其他颜色的色块识别

        注意：
            1、使用颜色识别，对环境光线变化有一定的要求，所以需要需要关闭自动曝光和白平衡自动增益
            2、由于初始化后关闭了自动曝光和白平衡自动增益，所以一定要放在使用场景下再运行程序，避免先后环境光不一样影响阈值判断

实验现象：
        程序运行后，摄像头初始化完成后，开始拍照，并对图像帧根据目标LAB阈值进行颜色识别，
        并打印出目标中心坐标，同时再IDE帧缓冲区显示框选 没目标色块
"""

import sensor, image, time
from machine import LED
import os


LEDB = LED("LED_BLUE")                  # 板载RGB红色LED

#--------------------------------------#初始化摄像头---------------------------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)     # 设置图像格式为 RGB565.
sensor.set_framesize(sensor.QVGA)       # 设置图像帧像素尺寸
sensor.skip_frames(time = 2000)         # 跳过1s，使新设置生效以及适应环境后关闭增益和白平衡自动设置
sensor.set_auto_gain(False)             # 关闭自动自动增益。默认开启的，在颜色识别中，一定要关闭白平衡。
sensor.set_auto_whitebal(False)         # 关闭白平衡。白平衡是默认开启的，在颜色识别一定要关闭。

clock = time.clock() # 追踪帧率

#--------------------------- 目标色块的LAB阈值元组列表 -------------------------------
target_thresholds = [
                (7, 31, 3, 46, 2, 26)   # 红色目标，最多可添加32组
        ]

def find_max(blob_all):# 获取相同色块中的最大的色块
    max_size=0
    for blob in blob_all:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob



while True:
    clock.tick()

    img = sensor.snapshot()                                         # 捕获一帧图像
    # 可能用到的img.find_blobs()参数:
    # thresholds:目标色块LAB阈值元组列表, 必须要设置的
    # roi=感兴区域四元组(x,y,w,h):如果为空则表示，对整个img帧图像进行搜索查找
    # x_stride=2, y_stride=1:在搜索blob时要跳过的x，y像素数,如果已知blob的大小，则可以可以借助其忽略小于目标的x,y像素数，以加快搜索速度
    # area_threshold=10:如果blob的边界框面积小于 area_threshold ，则过滤掉它。
    # pixels_threshold=10:如果blob的像素计数少于 pixels_threshold ，则过滤掉它。
    # merge=False: merge 如果为 True，则会合并所有未被过滤掉且其边界矩形相互交叉的斑点。
    # margin:可用于在交叉测试期间增加或减少斑点边界矩形的大小。例如，当 margin 为 1 时，边界矩形相距 1 像素的斑点将被合并。

    blobs = img.find_blobs(target_thresholds,pixels_threshold=25, area_threshold=30, merge=True)     #像素阈值25，面积阈值25，可接受的最小色块大小
    if blobs:                                                       # 找到追踪目标
        blob = find_max(blobs)                                      # 挑选最大的一个
        img.draw_rectangle(blob.rect(),color=( 0,255, 0))           # 根据色块边界框 框选目标
        img.draw_cross(blob.cx(), blob.cy(),color=(0, 0, 255))      # 根据色块中心位置画十字
        print(f"目标位置:{blob.cx(), blob.cy()}")

    LEDB.toggle()
    print(f"fps:{clock.fps():.2f}")

