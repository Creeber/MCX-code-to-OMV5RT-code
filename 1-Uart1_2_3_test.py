# -*- coding: utf-8 -*-

# LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
"""
    @File    :   Test_Uart1_2_3.py
    @Company :   龙邱科技
    @Platform:   LQ-OMV-RT
    @Desc    :   UART串口通信测试，外置串口1-3，收发测试
    @Author  :   LQ-008
    @Date    :   2024-10-14
    @Version :   V1.0.0
    @Note    :
        - 依赖库: OpenMV  machine LED，Pin
        - UART(1)--> TX:P4, RX:P5
          UART(2)--> TX:P9, RX:P10
          UART(3)--> TX:P1, RX:P3
        - 适用硬件: LQ-OMV-RT，LQ-MV5-RT
          详情了解：https://item.taobao.com/item.htm?id=850456445452
        - 运行现象：借助如USB-TTL工具连接OMV-RT的串口，运行程序
          三个串口各自发送一条 Hello World字符串，
          如果串口1收到字符 'B',IDE串行终端打印收到的数据，同时蓝色LDE状态翻转
    @Copyright © 2024 龙邱科技. All rights reserved.
"""
# QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ

import time
from machine import UART,LED

LEDB = LED("LED_BLUE")
LEDB.off()

# Always pass UART 1，2, 3 for the UART number OMV-RT Cam.
# The second argument is the UART baud rate.
uart1 = UART(1, 115200, timeout_char=200)      # TX:P4,RX:P5   --I2C(1)  共用管脚功能2选1
uart2 = UART(2, 115200, timeout_char=200)      # TX:P9,RX:P10
uart3 = UART(3, 115200, timeout_char=200)      # TX:P1,RX:P3

uart1.write("Hello World from UART1!\r")
uart2.write("Hello World from UART2!\r")
uart3.write("Hello World from UART3!\r")

# time.sleep_ms(500)
print("Masg Send OK")

while True:
    # 串口收发 UART2/3 同UART1 方法
    if uart1.any():                              # 判断是否接收到数据
        Rdate = uart1.read().decode().strip()    # 将接收到的消息提取出来
        print("RecX:",Rdate)                     # 在OpenMV IDE的串行终端中打印
        uart1.write(Rdate)                       # 将接收到的消息发回去
        if Rdate == 'B':                         # 如果接收到 字符 'B'
            LEDB.toggle()                        # 翻转 LEDB的状态
            print('LEDB.toggle() OK')            # 在串行终端中显示OK

