import time, os, sys
import math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA
from machine import Pin
# UART初始化
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

# 状态机相关变量
state = 0
last_button_state = 0
# 按键初始化（以53脚为例，请根据实际连线修改）
fpioa.set_function(53, FPIOA.GPIO53)
button = Pin(53, Pin.IN, Pin.PULL_DOWN)  # 下拉输入

sensor_id = 2
sensor = None
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

# ROI边缘宽度
ROI_MARGIN = 20
ROI_X = ROI_MARGIN
ROI_Y = ROI_MARGIN
ROI_W = DISPLAY_WIDTH - 2 * ROI_MARGIN
ROI_H = DISPLAY_HEIGHT - 2 * ROI_MARGIN

# 宽高比参数
MIN_ASPECT_RATIO = 0.5    # 最小宽高比
MAX_ASPECT_RATIO = 2.0    # 最大宽高比

class TargetTracker:
    def __init__(self):
        self.current_target = None
        self.last_target = None

    def update(self, candidates):
        if not candidates:
            self.current_target = None
            return None
        if self.current_target is None:
            selected = max(candidates, key=lambda c: c['area'])
            self.current_target = selected
            self.last_target = selected
            return selected
        if self.last_target:
            last_x, last_y = self.last_target['center_x'], self.last_target['center_y']
            selected = min(candidates, key=lambda c:
                (c['center_x'] - last_x)**2 + (c['center_y'] - last_y)**2)
            self.current_target = selected
        self.last_target = self.current_target
        return self.current_target

def find_max(rects):
    max_size = 0
    max_rect = None
    for rect in rects:
        rect_tuple = rect.rect()
        area = rect_tuple[2] * rect_tuple[3]
        if area > max_size:
            max_rect = rect
            max_size = area
    return max_rect

def calculate_perspective_center(corners):
    if len(corners) < 4:
        return None, None
    x1, y1 = corners[0]
    x3, y3 = corners[2]
    x2, y2 = corners[1]
    x4, y4 = corners[3]
    denom = (x1 - x3) * (y2 - y4) - (y1 - y3) * (x2 - x4)
    if abs(denom) < 1e-6:
        return sum(c[0] for c in corners) // 4, sum(c[1] for c in corners) // 4
    px = ((x1*y3 - y1*x3)*(x2 - x4) - (x1 - x3)*(x2*y4 - y2*x4)) / denom
    py = ((x1*y3 - y1*x3)*(y2 - y4) - (y1 - y3)*(x2*y4 - y2*x4)) / denom
    return int(px), int(py)

def in_roi(corners):
    """检测所有角点是否都在ROI区域内"""
    for x, y in corners:
        if not (ROI_X <= x < ROI_X + ROI_W and ROI_Y <= y < ROI_Y + ROI_H):
            return False
    return True

def detect_rect_candidates(img):

    rects = img.find_rects(threshold=5000)
    candidates = []
    for rect in rects:
        corners = rect.corners()
        if len(corners) == 4 and in_roi(corners):
            center_x, center_y = calculate_perspective_center(corners)
            x_min = min(c[0] for c in corners)
            y_min = min(c[1] for c in corners)
            x_max = max(c[0] for c in corners)
            y_max = max(c[1] for c in corners)
            w = x_max - x_min
            h = y_max - y_min
            if h == 0 or w == 0:
                continue
            aspect_ratio = w / h if w > h else h / w
            if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                continue
            area = w * h
            candidates.append({
                'corners': corners,
                'center_x': center_x,
                'center_y': center_y,
                'area': area,
            })
    return candidates

try:
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()

    tracker = TargetTracker()

    while True:
        os.exitpoint()

        # -------- 按键状态机+消抖 --------
        current_button_state = button.value()
        if current_button_state == 1 and last_button_state == 0:
            import utime
            utime.sleep_ms(30)  # 消抖延时
            if button.value() == 1:  # 再次确认
                state += 1
                if state == 1:
                    uart.write("f,1\n")
                    print("f,1\n")
                elif state == 2:
                    uart.write("f,2\n")
                    print("f,2")
                elif state ==3:
                    uart.write("f,3\n")
                    print("f,3")
                if state >= 3:
                    state = 0
        last_button_state = current_button_state

        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        gray_img = img.to_grayscale()
        # 只处理ROI区域
        gray_img.gamma_corr(3)
        binary_img = gray_img.binary([(160, 255)], roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
        # 绘制ROI区域用于调试



#        img.draw_rectangle((ROI_X, ROI_Y, ROI_W, ROI_H), color=(255, 0, 0), thickness=2)

        candidates = detect_rect_candidates(binary_img)
        current_target = tracker.update(candidates)

        if current_target:
            corners = current_target['corners']
            center_x = current_target['center_x']
            center_y = current_target['center_y']
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
#            for i, point in enumerate(corners):
#                img.draw_circle(int(point[0]), int(point[1]), 5, color=colors[i], thickness=2)
            for i in range(4):
                start = corners[i]
                end = corners[(i + 1) % 4]
#                img.draw_line(int(start[0]), int(start[1]), int(end[0]), int(end[1]), color=(1, 147, 230), thickness=2)
            img.draw_cross(center_x, center_y, size=10, color=(255, 0, 0), thickness=2)
#            img.draw_line(int(corners[0][0]), int(corners[0][1]), int(corners[2][0]), int(corners[2][1]), color=(0, 255, 0), thickness=1)
#            img.draw_line(int(corners[1][0]), int(corners[1][1]), int(corners[3][0]), int(corners[3][1]), color=(0, 255, 0), thickness=1)
            message = "t,{},{}\n".format(center_x, center_y)
            uart.write(message)
        else:
            rects = binary_img.find_rects(threshold=10000)
            if rects:
                rect = find_max(rects)
                rect_tuple = rect.rect()
                w = rect_tuple[2]
                h = rect_tuple[3]
                if h == 0 or w == 0:
                    continue
                aspect_ratio = w / h if w > h else h / w
                if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                    continue
                center_x = int(rect_tuple[0] + w/2)
                center_y = int(rect_tuple[1] + h/2)
#                img.draw_cross(center_x, center_y, size=5, color=(255, 0, 0))
#                img.draw_rectangle(rect_tuple, color=(1, 147, 230), thickness=2)
                message = "t,{},{}\n".format(center_x, center_y)
                uart.write(message)

        Display.show_image(img)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(10)
    MediaManager.deinit()
