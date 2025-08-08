import os
import ujson
import aicube
from media.sensor import *
from media.display import *
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import utime
from machine import UART
from machine import FPIOA

# 初始化UART
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
display_mode="lcd"
if display_mode=="lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1080, 16)
OUT_RGB888P_HEIGH = 720

# 颜色盘
color_four = [(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
        (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
        (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
        (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
        (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157),
        (255, 110, 76, 0), (255, 174, 57, 255), (255, 199, 100, 0), (255, 72, 0, 118),
        (255, 255, 179, 240), (255, 0, 125, 92), (255, 209, 0, 151), (255, 188, 208, 182),
        (255, 0, 220, 176), (255, 255, 99, 164), (255, 92, 0, 73), (255, 133, 129, 255),
        (255, 78, 180, 255), (255, 0, 228, 0), (255, 174, 255, 243), (255, 45, 89, 255),
        (255, 134, 134, 103), (255, 145, 148, 174), (255, 255, 208, 186),
        (255, 197, 226, 255), (255, 171, 134, 1), (255, 109, 63, 54), (255, 207, 138, 255),
        (255, 151, 0, 95), (255, 9, 80, 61), (255, 84, 105, 51), (255, 74, 65, 105),
        (255, 166, 196, 102), (255, 208, 195, 210), (255, 255, 109, 65), (255, 0, 143, 149),
        (255, 179, 0, 194), (255, 209, 99, 106), (255, 5, 121, 0), (255, 227, 255, 205),
        (255, 147, 186, 208), (255, 153, 69, 1), (255, 3, 95, 161), (255, 163, 255, 0),
        (255, 119, 0, 170), (255, 0, 182, 199), (255, 0, 165, 120), (255, 183, 130, 88),
        (255, 95, 32, 0), (255, 130, 114, 135), (255, 110, 129, 133), (255, 166, 74, 118),
        (255, 219, 142, 185), (255, 79, 210, 114), (255, 178, 90, 62), (255, 65, 70, 15),
        (255, 127, 167, 115), (255, 59, 105, 106), (255, 142, 108, 45), (255, 196, 172, 0),
        (255, 95, 54, 80), (255, 128, 76, 255), (255, 201, 57, 1), (255, 246, 0, 122),
        (255, 191, 162, 208)]

root_path="/sdcard/mp_deployment_source/"
config_path=root_path+"deploy_config.json"
deploy_conf={}
debug_mode=1
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
            if 0.8< ratio < 2.5 and area>7500  :  # 宽高比必须在1.5到2之间
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
def detect_corners(img,a,b,c,d):
    # 转换为灰度图像
    gray_img = img.to_grayscale()

    gray_img.gamma_corr(3)
    # 二值化处理
    binary_img = gray_img.binary([(160, 255)])  # 调整阈值100,255
#    binary_img.draw_rectangle(a, b, c, d, color=(0, 255, 0), thickness=2)
#    binary_img.gaussian(3)

    # 寻找所有矩形
    if a-30<0:
        a = 0
    else :
        a = a-30
    if b-30<0:
        b = 0
    else :
        b = b-30              
    if a+c+180<800:
        c= c+180
    else :
        c= 800-a
    if b+d+180<480:
        d= d+180
    else :
        d= 480-b
        
    rects = binary_img.find_rects((a,b,c,d),threshold=1000)
#    binary_img.draw_rectangle(a, b, c, d, color=(0, 255, 0), thickness=2)
   
#    for rect in rects:
#        print("赢了")
#         # 若想获取更详细的四个顶点，可使用 rect.corners()，该函数会返回一个有四个元祖的列表，每个元组代表矩形的四个顶点，从左上角开始，按照顺时针排序。
#        binary_img.draw_rectangle(rect.rect(), color=(225, 0, 230), thickness=3)  # 绘制线段
#    Display.show_image(binary_img,0, 0,Display.LAYER_OSD1)

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
class ScopedTiming:
    def __init__(self, info="", enable_profile=True):
        self.info = info
        self.enable_profile = enable_profile

    def __enter__(self):
        if self.enable_profile:
            self.start_time = time.time_ns()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enable_profile:
            elapsed_time = time.time_ns() - self.start_time
            print(f"{self.info} took {elapsed_time / 1000000:.2f} ms")

def read_deploy_config(config_path):
    # 打开JSON文件以进行读取deploy_config
    with open(config_path, 'r') as json_file:
        try:
            # 从文件中加载JSON数据
            config = ujson.load(json_file)

            # 打印数据（可根据需要执行其他操作）
            #print(config)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

def detection():
    print("det_infer start")
    # 使用json读取内容初始化部署变量
    deploy_conf=read_deploy_config(config_path)
    kmodel_name=deploy_conf["kmodel_path"]
    labels=deploy_conf["categories"]
    confidence_threshold= deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size=deploy_conf["img_size"]
    num_classes=deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH,OUT_RGB888P_HEIGH]
    strides = [8,16,32]

    # 计算padding值
    ori_w = OUT_RGB888P_WIDTH;
    ori_h = OUT_RGB888P_HEIGH;
    width = kmodel_frame_size[0];
    height = kmodel_frame_size[1];
    ratiow = float(width) / ori_w;
    ratioh = float(height) / ori_h;
    if ratiow < ratioh:
        ratio = ratiow
    else:
        ratio = ratioh
    new_w = int(ratio * ori_w);
    new_h = int(ratio * ori_h);
    dw = float(width - new_w) / 2;
    dh = float(height - new_h) / 2;
    top = int(round(dh - 0.1));
    bottom = int(round(dh + 0.1));
    left = int(round(dw - 0.1));
    right = int(round(dw - 0.1));

    # init kpu and load kmodel
    kpu = nn.kpu()
    ai2d = nn.ai2d()
    kpu.load_kmodel(root_path+kmodel_name)
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT,
                                   nn.ai2d_format.NCHW_FMT,
                                   np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0,top,bottom,left,right], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel )
    ai2d_builder = ai2d.build([1,3,OUT_RGB888P_HEIGH,OUT_RGB888P_WIDTH], [1,3,height,width])

    # 初始化并配置sensor
    sensor = Sensor()
    sensor.reset()
    # 设置镜像
    sensor.set_hmirror(False)
    # 设置翻转
    sensor.set_vflip(False)
    # 通道0直接给到显示VO，格式为YUV420
    sensor.set_framesize(width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT)
    sensor.set_pixformat(sensor.RGB565)
    # 通道2给到AI做算法处理，格式为RGB888
    sensor.set_framesize(width = OUT_RGB888P_WIDTH , height = OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    # 绑定通道0的输出到vo
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
   

    try:
        # media初始化
        MediaManager.init()
        # 启动sensor
        sensor.run()
        rgb888p_img = None
        ai2d_input_tensor = None
        data = np.ones((1,3,width,height),dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        while  True:
            with ScopedTiming("total",debug_mode > 0):
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
                img1 = sensor.snapshot(chn=CAM_CHN_ID_0)
                
                # for rgb888planar
                if rgb888p_img.format() == image.RGBP888:
                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)
                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)

                    # set input
                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    # run kmodel
                    kpu.run()
                    # get output
                    results = []
                    for i in range(kpu.outputs_size()):
                        out_data = kpu.get_output_tensor(i)
                        result = out_data.to_numpy()
                        result = result.reshape((result.shape[0]*result.shape[1]*result.shape[2]*result.shape[3]))
                        del out_data
                        results.append(result)
                    gc.collect()

                    # postprocess
                    if model_type == "AnchorBaseDet":
                        det_boxes = aicube.anchorbasedet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, anchors, nms_option)
                    elif model_type == "GFLDet":
                        det_boxes = aicube.gfldet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                    else:
                        det_boxes = aicube.anchorfreedet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)

                    if det_boxes:
                        for det_boxe in det_boxes:
                            x1, y1, x2, y2 = det_boxe[2],det_boxe[3],det_boxe[4],det_boxe[5]
                            w = float(x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
                            h = float(y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH
                            corners = detect_corners(img1,int(x1*0.6407),int(y1*0.6067),int(w*1.3),int(h*1.3))
                            if corners:
                                # 提取角点坐标
                                corners = [(int(c[0]), int(c[1])) for c in corners]

                                # 绘制角点和矩形轮廓
                                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
                                for i, point in enumerate(corners):
                                    img1.draw_circle(point[0], point[1], 5, color=colors[i], thickness=2)

                                # 绘制矩形轮廓
                                for i in range(4):
                                    start = corners[i]
                                    end = corners[(i + 1) % 4]
                                    img1.draw_line(start[0], start[1], end[0], end[1], color=(1, 147, 230), thickness=2)

                                # 计算精确中心点（使用对角线交点）
                                center_x, center_y = calculate_perspective_center(corners)

                                if center_x is not None:
                                    # 绘制中心点和对角线
                                    img1.draw_cross(center_x, center_y, size=10, color=(255, 0, 0), thickness=2)
#                                    img1.draw_line(corners[0][0], corners[0][1], corners[2][0], corners[2][1], color=(0, 255, 0), thickness=1)
#                                    img1.draw_line(corners[1][0], corners[1][1], corners[3][0], corners[3][1], color=(0, 255, 0), thickness=1)

                                    # 发送中心点坐标
                                    message = "{},{}\n".format(center_x, center_y)
                                    uart.write(message)
                                    
#                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                    Display.show_image(img1,0, 0,Display.LAYER_OSD1)
                    gc.collect()
                rgb888p_img = None
    except Exception as e:
        print(f"An error occurred during buffer used: {e}")
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        del ai2d_input_tensor
        del ai2d_output_tensor
        #停止摄像头输出
        sensor.stop()
        #去初始化显示设备
        Display.deinit()
        #释放媒体缓冲区
        MediaManager.deinit()
        gc.collect()
        time.sleep(1)
        nn.shrink_memory_pool()
    print("det_infer end")
    return 0


if __name__=="__main__":
    detection()
