import time, os, sys
import math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA

# 初始化UART
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

sensor_id = 2
sensor = None
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

def find_max(rects):
    max_size = 0
    max_rect = None
    for rect in rects:
        rect_tuple = rect.rect()
        w = rect_tuple[2]
        h = rect_tuple[3]

        # 添加宽高比检查：只有当1.5 < 宽高比 < 2时才考虑
        if min(w, h) > 0:
            ratio =w / h
            area = w * h
            if 0.5< ratio < 2.5  and area >7000 :  # 宽高比必须在1.5到2之间
                if area > max_size:
                    max_rect = rect
                    max_size = area
    return max_rect

# 精确计算透视畸变下的中心点
def calculate_perspective_center(corners):
    """
    计算透视畸变下矩形的真实中心点
    参数: corners - 矩形的四个角点 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    返回: (center_x, center_y) - 真实中心点
    """
    if len(corners) < 4:
        return None, None

    # 计算两条对角线的交点
    # 对角线1: 点1到点3
    x1, y1 = corners[0]
    x3, y3 = corners[2]

    # 对角线2: 点2到点4
    x2, y2 = corners[1]
    x4, y4 = corners[3]

    # 计算交点（中心点）
    denom = (x1 - x3) * (y2 - y4) - (y1 - y3) * (x2 - x4)
    if abs(denom) < 1e-6:  # 避免除零错误
        return sum(c[0] for c in corners) // 4, sum(c[1] for c in corners) // 4

    # 交点公式
    px = ((x1*y3 - y1*x3)*(x2 - x4) - (x1 - x3)*(x2*y4 - y2*x4)) / denom
    py = ((x1*y3 - y1*x3)*(y2 - y4) - (y1 - y3)*(x2*y4 - y2*x4)) / denom

    return int(px), int(py)

# 检测矩形角点
def detect_corners(img):
    # 转换为灰度图像
    gray_img = img.to_grayscale()

    gray_img.gamma_corr(2)
    # 二值化处理
    binary_img = gray_img.binary([(150, 255)])  # 调整阈值100,255
    #binary_img.gaussian(2)
    # 寻找所有矩形
    rects = binary_img.find_rects(threshold=5000)

    if not rects or len(rects) < 1:
        return None

    # 找到最大的矩形（已包含宽高比检查）
    rect = find_max(rects)

    # 如果没有符合宽高比的矩形，返回None
    if rect is None:
        return None

    # 获取矩形的四个角点
    corners = rect.corners()

    # 确保有4个角点
    if len(corners) != 4:
        return None

    # 对角点进行排序：左上、右上、右下、左下
    # 1. 找到左上角点（x+y最小）
    corners = sorted(corners, key=lambda c: c[0] + c[1])
    tl = corners[0]

    # 2. 找到右下角点（x+y最大）
    br = corners[-1]

    # 3. 剩下的两个点
    others = [c for c in corners if c != tl and c != br]

    # 4. 区分右上和左下
    if len(others) == 2:
        # 右上角点（x较大）
        if others[0][0] > others[1][0]:
            tr = others[0]
            bl = others[1]
        else:
            tr = others[1]
            bl = others[0]
        return [tl, tr, br, bl]

    return corners

try:
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()

    while True:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        # 检测矩形角点
        corners = detect_corners(img)

        if corners:
            # 提取角点坐标
            corners = [(int(c[0]), int(c[1])) for c in corners]

            # 绘制角点和矩形轮廓
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
            for i, point in enumerate(corners):
                img.draw_circle(point[0], point[1], 5, color=colors[i], thickness=2)

            # 绘制矩形轮廓
            for i in range(4):
                start = corners[i]
                end = corners[(i + 1) % 4]
                img.draw_line(start[0], start[1], end[0], end[1], color=(1, 147, 230), thickness=2)

            # 计算精确中心点（使用对角线交点）
            center_x, center_y = calculate_perspective_center(corners)

            if center_x is not None:
                # 绘制中心点和对角线
                img.draw_cross(center_x, center_y, size=10, color=(255, 0, 0), thickness=2)
                img.draw_line(corners[0][0], corners[0][1], corners[2][0], corners[2][1], color=(0, 255, 0), thickness=1)
                img.draw_line(corners[1][0], corners[1][1], corners[3][0], corners[3][1], color=(0, 255, 0), thickness=1)

                # 发送中心点坐标
                message = "t,{},{}\n".format(center_x, center_y)
                if center_x !=-1:
                    uart.write(message)
                    print("t,{},{}\n".format(center_x, center_y))

        Display.show_image(img)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
