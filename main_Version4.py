# 导入必要的库
import sensor
import image
import time
from machine import UART, LED
import struct
import gc
import math
import os

"""
@file OMV-RT 基于色块识别的赛道循迹系统
集成了:
1. 色块识别 - 识别目标色块并输出位置
2. 赛道检测 - 判断目标是否在赛道内及转弯类型
3. 串口通信 - 与主控板进行数据交换

作者: Creeber
最后更新: 2025-05-29 19:16:14
"""

# ------------------- 配置部分 -------------------
# 传感器配置
SENSOR_SETTINGS = {
    'pixformat': sensor.RGB565,
    'framesize': sensor.QQVGA,      # 改为QQVGA以提高帧率
    'fps': 30,
    'auto_gain': False,
    'auto_whitebal': False
}

# 色块检测参数
BLOB_SETTINGS = {
    'THRESHOLDS': [
        (7, 31, 3, 46, 2, 26),    # 红色目标
        (4, 44, 8, 120, 6, 110)   # 第二组阈值
    ],
    'PIXELS_THRESHOLD': 25,        # 像素阈值
    'AREA_THRESHOLD': 30,          # 面积阈值
    'MERGE': True                  # 允许合并色块
}

# 颜色追踪参数
COLOR_SETTINGS = {
    'ALLOW_FAIL_PER': 10,    # 容错率
    'ITERATE_NUM': 5,        # 迭代次数
    'H_RANGE': 60,          # 色调范围
    'S_RANGE': 80,          # 饱和度范围
    'L_RANGE': 50,          # 亮度范围
}

# 颜色阈值
COLOR_THRESHOLDS = {
    'WHITE': {
        'UP': 120,
        'DOWN': 0
    },
    'RED': {
        'UP': 255,
        'DOWN': 185
    },
    'BLUE': {
        'UP': 175,
        'DOWN': 125
    }
}

# 赛道检测参数
TRACK_SETTINGS = {
    'EDGE': 2,                      # 边界（需>=2）
    'ALLOW_PERCENT': 0.6,           # 判断箱子在赛道的比例阈值
    'ALLOW_WHITE_ROW': 5,          # 允许的白色行数
    'ALLOW_INCREASE': 8,           # 允许的递增次数
    'DIUXIAN_BOUNDARY': 200,       # 丢线底边界
    'CANCHA_THRESHOLD': 10,        # 残差阈值
    'CANCHA_K': 1.0,              # 残差系数
    'SMALL_CORNER_THRESHOLD': 8    # 小弯道判断阈值
}

# 串口通信参数
UART_SETTINGS = {
    'PORT': 1,          # UART端口号
    'BAUDRATE': 115200, # 波特率
    'BITS': 8,         # 数据位
    'PARITY': None,    # 校验位
    'STOP': 1,         # 停止位
}

# 通信协议标志位
COMM_FLAGS = {
    'HEADER1': 0xAA,
    'HEADER2': 0xBB,
    'FOOTER': 0xDD,
    'PARAM_INIT': 0xAB,
    'CORRECT': 0xAC
}

# 赛道状态枚举
class TrackStatus:
    STRAIGHT = 0    # 直道
    S_LEFT = 1      # 小左弯
    S_RIGHT = 2     # 小右弯
    B_LEFT_I = 3    # 大左弯内弯
    B_RIGHT_I = 4   # 大右弯内弯
    B_LEFT_O = 5    # 大左弯外弯
    B_RIGHT_O = 6   # 大右弯外弯

# LED指示灯配置
LED_SETTINGS = {
    'BLUE': "LED_BLUE",    # 蓝色LED引脚
    'BLINK_RATE': 500      # LED闪烁频率(ms)
}

# 调试参数
DEBUG_SETTINGS = {
    'SHOW_FPS': True,           # 是否显示帧率
    'SHOW_POSITION': True,      # 是否显示目标位置
    'DRAW_MARKERS': True,       # 是否绘制标记
    'PRINT_MEMORY': False,      # 是否打印内存使用情况
    'LOG_LEVEL': 'INFO'         # 日志级别：DEBUG/INFO/WARNING/ERROR
}

# 内存管理参数
MEMORY_SETTINGS = {
    'GC_COLLECT_FREQ': 10,      # 垃圾回收频率(每N帧)
    'MIN_FREE_MEM': 10000,      # 最小可用内存(bytes)
    'WARNING_MEM': 20000        # 内存警告阈值(bytes)
}

# 错误处理参数
ERROR_SETTINGS = {
    'MAX_RETRIES': 3,           # 最大重试次数
    'RETRY_DELAY': 100,         # 重试延迟(ms)
    'ERROR_LED_PATTERN': [      # LED错误指示模式
        (True, 200),            # (LED状态, 持续时间ms)
        (False, 200),
        (True, 200),
        (False, 600)
    ]
}

# 系统信息
SYSTEM_INFO = {
    'VERSION': '1.0.0',
    'AUTHOR': 'Creeber',
    'CREATED_DATE': '2025-05-29',
    'LAST_UPDATE': '2025-05-29 19:30:17'
}

# 图像处理参数
IMAGE_SETTINGS = {
    'ROI_MARGIN': 10,          # ROI边界裕度
    'MIN_CONTOUR_AREA': 100,   # 最小轮廓面积
    'MAX_CONTOUR_AREA': 10000, # 最大轮廓面积
    'FILTER_KERNEL_SIZE': 3,   # 滤波核大小
    'EDGE_THRESHOLD': 30       # 边缘检测阈值
}

# 性能优化参数
PERFORMANCE_SETTINGS = {
    'SKIP_FRAMES': 0,          # 跳帧数
    'ROI_ENABLED': True,       # 是否启用ROI
    'ADAPTIVE_FPS': True,      # 自适应帧率
    'MIN_FPS': 15,            # 最低帧率
    'TARGET_FPS': 30          # 目标帧率
}

# ------------------- 赛道检测部分 -------------------
class TrackResult:
    def __init__(self):
        self.x1 = 0          # 目标框左上角x坐标
        self.y1 = 0          # 目标框左上角y坐标
        self.x2 = 0          # 目标框右下角x坐标
        self.y2 = 0          # 目标框右下角y坐标
        self.box_in_track = False  # 是否在赛道内
        self.draw_debug = True    # 是否绘制调试信息

class TrackDetector:
    def __init__(self):
        # 基础参数
        self.status = TrackStatus.STRAIGHT
        self.edge = TRACK_SETTINGS['EDGE']
        self.allow_percent = TRACK_SETTINGS['ALLOW_PERCENT']

        # 图像尺寸
        self.img_width = sensor.width()
        self.img_height = sensor.height()

        # 赛道边界数组
        self.left_line = [self.edge] * self.img_height
        self.right_line = [self.img_width - self.edge] * self.img_height

        # 统计变量
        self.left_white_count = 0
        self.right_white_count = 0
        self.left_increase_count = 0
        self.right_increase_count = 0

        # 色块检测相关
        self.found_blob = None
        self.blob_thresholds = BLOB_SETTINGS['THRESHOLDS']

    def find_max_blob(self, blobs):
        """获取最大的色块"""
        max_size = 0
        max_blob = None
        for blob in blobs:
            if blob[2] * blob[3] > max_size:
                max_blob = blob
                max_size = blob[2] * blob[3]
        return max_blob

    def detect(self, img):
        """主要的检测函数，包含色块检测和赛道检测"""
        result = TrackResult()

        # 色块检测
        blobs = img.find_blobs(
            self.blob_thresholds,
            pixels_threshold=BLOB_SETTINGS['PIXELS_THRESHOLD'],
            area_threshold=BLOB_SETTINGS['AREA_THRESHOLD'],
            merge=BLOB_SETTINGS['MERGE']
        )

        if blobs:
            # 找到最大的色块
            self.found_blob = self.find_max_blob(blobs)

            # 获取矩形框坐标
            x, y, w, h = self.found_blob.rect()

            # 更新结果坐标
            result.x1 = x           # 左上角x坐标
            result.y1 = y           # 左上角y坐标
            result.x2 = x + w       # 右下角x坐标
            result.y2 = y + h       # 右下角y坐标

            # 在图像上绘制标记
            if result.draw_debug:
                img.draw_rectangle(self.found_blob.rect(), color=(0, 255, 0))
                img.draw_cross(self.found_blob.cx(), self.found_blob.cy(), color=(0, 0, 255))

            # 检查是否在赛道内
            result.box_in_track = self._check_box_in_track(img)

            # 如果在赛道内，进行转弯判断
            if result.box_in_track:
                self.judge_corner(img)

        return result

    def _check_box_in_track(self, img):
        """检查箱子是否在赛道内"""
        if not self.found_blob:
            return False

        row_blue_count = 0
        box_width = abs(self.x2 - self.x1)

        # 扫描箱子下方区域
        for col in range(self.x1, self.x2):
            col_blue_count = 0
            if row_blue_count >= box_width * self.allow_percent:
                return False

            for row in range(self.y2 + 5, self.img_height):
                # 获取像素LAB值并判断是否为赛道颜色
                pixel = img.get_pixel(col, row)
                if self._is_track_color(pixel):
                    col_blue_count += 1
                if col_blue_count >= 2:
                    row_blue_count += 1
                    break

        return True

    def judge_corner(self, img):
        """判断转弯类型"""
        # 重置计数器
        self.left_white_count = 0
        self.right_white_count = 0
        self.left_increase_count = 0
        self.right_increase_count = 0

        # 扫描边界
        self._scan_boundaries(img)

        # 判断转弯类型
        self._determine_corner_type()

        # 处理特殊情况
        self._handle_special_cases(img)

    def _scan_boundaries(self, img):
        """扫描赛道边界"""
        for row in range(self.img_height - 1, -1, -1):
            # 扫描左边界
            self._scan_left_boundary(img, row)
            # 扫描右边界
            self._scan_right_boundary(img, row)
            # 统计边界信息
            self._count_boundary_stats(row)

    def _scan_left_boundary(self, img, row):
        """扫描左边界"""
        last_col = self.left_line[row + 1] if row < self.img_height - 1 else self.edge
        found_boundary = False

        for col in range(last_col, self.img_width - 5):
            colors = []
            # 获取5个点的颜色状态
            for i in range(-2, 3):
                if 0 <= col + i < self.img_width:
                    pixel = img.get_pixel(col + i, row)
                    colors.append(self._is_track_color(pixel))
                else:
                    colors.append(False)

            # 左边界模式: 00111
            if (not colors[0] and not colors[1] and
                colors[2] and colors[3] and colors[4]):
                self.left_line[row] = col
                found_boundary = True
                break

        if not found_boundary:
            self.left_line[row] = self.edge

    def _scan_right_boundary(self, img, row):
        """扫描右边界"""
        last_col = self.right_line[row + 1] if row < self.img_height - 1 else self.img_width - self.edge
        found_boundary = False

        for col in range(last_col, 4, -1):
            colors = []
            # 获取5个点的颜色状态
            for i in range(-2, 3):
                if 0 <= col + i < self.img_width:
                    pixel = img.get_pixel(col + i, row)
                    colors.append(self._is_track_color(pixel))
                else:
                    colors.append(False)

            # 右边界模式: 11100
            if (colors[0] and colors[1] and colors[2] and
                not colors[3] and not colors[4]):
                self.right_line[row] = col
                found_boundary = True
                break

        if not found_boundary:
            self.right_line[row] = self.img_width - self.edge

    def _count_boundary_stats(self, row):
        """统计边界信息"""
        # 统计边界丢线
        if row >= self.y1 and row <= self.y2:
            if self.left_line[row] <= self.edge:
                self.left_white_count += 1
            if self.right_line[row] >= self.img_width - self.edge:
                self.right_white_count += 1

        # 统计边界递增
        if row > self.y1 + (self.y2 - self.y1) * 0.2:
            if (self.left_line[row] < self.left_line[row + 1] and
                self.left_line[row] > self.edge):
                self.left_increase_count += 1
            if (self.right_line[row] > self.right_line[row + 1] and
                self.right_line[row] < self.img_width - self.edge):
                self.right_increase_count += 1

    def _determine_corner_type(self):
        """确定转弯类型"""
        if (self.left_white_count >= TRACK_SETTINGS['ALLOW_WHITE_ROW'] and
            self.right_white_count >= TRACK_SETTINGS['ALLOW_WHITE_ROW']):

            if (self.left_increase_count <= TRACK_SETTINGS['ALLOW_INCREASE'] and
                self.right_increase_count <= TRACK_SETTINGS['ALLOW_INCREASE']):

                if (self.right_white_count > self.left_white_count or
                    self.right_increase_count > self.left_increase_count):
                    self.status = TrackStatus.B_RIGHT_I
                elif (self.right_white_count < self.left_white_count or
                      self.left_increase_count > self.right_increase_count):
                    self.status = TrackStatus.B_LEFT_I
                else:
                    self.status = TrackStatus.STRAIGHT

            elif (self.left_increase_count <= TRACK_SETTINGS['ALLOW_INCREASE'] and
                  self.right_increase_count > TRACK_SETTINGS['ALLOW_INCREASE']):
                self.status = TrackStatus.B_RIGHT_I

            elif (self.left_increase_count > TRACK_SETTINGS['ALLOW_INCREASE'] and
                  self.right_increase_count <= TRACK_SETTINGS['ALLOW_INCREASE']):
                self.status = TrackStatus.B_LEFT_I

        elif self.left_white_count >= TRACK_SETTINGS['ALLOW_WHITE_ROW']:
            self.status = TrackStatus.B_LEFT_I

        elif self.right_white_count >= TRACK_SETTINGS['ALLOW_WHITE_ROW']:
            self.status = TrackStatus.B_RIGHT_I

    def _handle_special_cases(self, img):
        """处理特殊情况"""
        # 检查是否需要判断小弯道
        if self.status == TrackStatus.STRAIGHT:
            if self.left_increase_count >= TRACK_SETTINGS['ALLOW_INCREASE']:
                self._check_small_corner('left')
            elif self.right_increase_count >= TRACK_SETTINGS['ALLOW_INCREASE']:
                self._check_small_corner('right')

    def _check_small_corner(self, side):
        """检查小弯道"""
        chafen_sum = 0
        start_row = int(self.y1 + (self.y2 - self.y1) * 0.2)

        for row in range(self.img_height - 2, start_row, -1):
            if side == 'left':
                yijiechafen = abs(self.right_line[row] - self.right_line[row + 1])
            else:
                yijiechafen = abs(self.left_line[row] - self.left_line[row + 1])

            if yijiechafen <= 1:
                yijiechafen = 0
            chafen_sum += yijiechafen

        # 根据差分和判断是否为小弯道
        if chafen_sum <= TRACK_SETTINGS.get('SMALL_CORNER_THRESHOLD', 8):
            self.status = TrackStatus.STRAIGHT
        else:
            self.status = TrackStatus.S_LEFT if side == 'left' else TrackStatus.S_RIGHT

    def _is_track_color(self, pixel):
        """判断像素是否为赛道颜色
        这里使用简单的亮度阈值，您可以根据实际赛道颜色调整判断逻辑
        """
        r = (pixel >> 11) & 0x1F
        g = (pixel >> 5) & 0x3F
        b = pixel & 0x1F

        # 转换为亮度值
        luminance = (r * 77 + g * 150 + b * 29) >> 8

        # 返回是否为赛道颜色
        return luminance > 60  # 可以根据实际情况调整阈值

# ------------------- 主程序部分 -------------------
class MainController:
    def __init__(self):
        # 启用垃圾回收
        gc.enable()

        # 初始化LED
        self.led_blue = LED("LED_BLUE")

        # 初始化各个模块
        self._init_sensor()
        self.track_detector = TrackDetector()
        self.uart_comm = UartComm()

        # 状态变量
        self.clock = time.clock()
        self.is_running = True

    def _init_sensor(self):
        """初始化摄像头参数"""
        sensor.reset()
        sensor.set_pixformat(SENSOR_SETTINGS['pixformat'])
        sensor.set_framesize(SENSOR_SETTINGS['framesize'])
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(SENSOR_SETTINGS['auto_gain'])
        sensor.set_auto_whitebal(SENSOR_SETTINGS['auto_whitebal'])

    def run(self):
        """主循环"""
        print("System starting...")

        while self.is_running:
            try:
                self.clock.tick()

                # 主动回收内存
                gc.collect()

                # 捕获图像
                frame = sensor.snapshot()

                # 处理图像
                track_result = self.track_detector.detect(frame)

                # 如果检测到目标且在赛道内
                if track_result.box_in_track:
                    # 通过串口发送结果
                    self.uart_comm.send_track_result(
                        track_result,
                        self.track_detector.status
                    )

                # 检查串口命令
                self.check_uart_commands()

                # 打印调试信息
                if True:  # 调试信息开关
                    print(f"FPS: {self.clock.fps():.2f}")
                    if self.track_detector.found_blob:
                        print(f"目标位置: ({self.track_detector.found_blob.cx()}, {self.track_detector.found_blob.cy()})")

                # 切换LED状态
                self.led_blue.toggle()

            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep_ms(100)

def main():
    """入口函数"""
    try:
        controller = MainController()
        controller.run()
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        print("Program ended")

if __name__ == "__main__":
    main()
