import time
import os
import sys
import math
from media.sensor import *
from media.display import *
from media.media import *
from time import ticks_ms
from machine import FPIOA
from machine import TOUCH
from machine import PWM
from machine import Pin

sensor = None
pwm_h = None
pwm_v = None

blue = 90, 100, -11, 10, -60, 43
black = (0, 170)

#摄像头的物理中心 (分辨率: 320x240)
TARGET_POINT = (156, 113)

detect_counter = 0
lost_counter = 0
min_detect_frames = 2
min_lost_frames = 5
flag_detected = False

threshold_dict = {'black': [black]}
adjusting_threshold = False
current_threshold = list(black)

has_touch = False
try:
    tp = TOUCH(0)
    has_touch = True
except Exception as e:
    has_touch = False

try:
    trigger_pin = Pin(41, Pin.IN, Pin.PULL_UP)
except Exception as e:
    trigger_pin = Pin(41, Pin.IN)

try:
    laser_pin = Pin(37, Pin.OUT)
    laser_pin.value(0)
except Exception as e:
    print("激光引脚初始化失败:", e)
    laser_pin = None

#核心调参

H_ANGLE_MIN = 0
H_ANGLE_MAX = 270
DEFAULT_H_ANGLE = 270

V_ANGLE_MIN = 160
V_ANGLE_MAX = 210
DEFAULT_V_ANGLE = 186.0

SCAN_SPEED_H = 7.5

#纯线性 PID
KP_X = 0.025
KI_X = 0.004
KD_X = 0.040
KP_Y = 0.025
KI_Y = 0.004
KD_Y = 0.040

DEADZONE = 14
I_MAX = 6.0
MAX_TRACK_STEP = 6.5

#平滑滤波
EMA_ALPHA = 0.25
PREDICT_FACTOR = 0.7
REF_BOX_WIDTH = 100.0

SCAN_MIN_AREA = 2200
TRACK_MIN_AREA = 650

#双轴视差补偿
LASER_OFFSET_X_RATIO = 0.03
LASER_OFFSET_Y_RATIO = 0.001

BASE_OFFSET_X = 22
BASE_OFFSET_Y = -6


PERIOD_US = 20000
PULSE_MIN_US = 500
PULSE_MAX_US = 2500

current_h_angle = DEFAULT_H_ANGLE
current_v_angle = DEFAULT_V_ANGLE

integral_x = 0
integral_y = 0
last_dx = 0
last_dy = 0
pid_active = False

smooth_x = TARGET_POINT[0]
smooth_y = TARGET_POINT[1]
prev_raw_x = TARGET_POINT[0]
prev_raw_y = TARGET_POINT[1]

scan_direction = 1

debug_dx = 0
debug_dy = 0
debug_delta_h = 0.0
debug_delta_v = 0.0

def angle_to_duty_u16(angle):
    if angle < 0: angle = 0
    elif angle > 270: angle = 270
    pulse_us = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * angle / 270.0
    return int((pulse_us / PERIOD_US) * 65535)

def set_servo(pwm_obj, angle):
    if pwm_obj is not None:
        pwm_obj.duty_u16(angle_to_duty_u16(angle))

def vector_angle_diff(v1, v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    angle = math.atan2(det, dot) * (180 / math.pi)
    return abs(angle)

def get_line_intersection(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    A1 = y2 - y1
    B1 = x1 - x2
    C1 = A1 * x1 + B1 * y1
    A2 = y4 - y3
    B2 = x3 - x4
    C2 = A2 * x3 + B2 * y3
    det = A1 * B2 - A2 * B1
    if det == 0:
        return ((x1 + x3) / 2, (y1 + y3) / 2)
    else:
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        return (x, y)

try:
    print("camera_test_Ultimate_Linear_Track")
    fpioa = FPIOA()
    fpioa.set_function(42, FPIOA.PWM0)
    fpioa.set_function(52, FPIOA.PWM4)
    pwm_h = PWM(0, freq=50)
    pwm_v = PWM(4, freq=50)

    set_servo(pwm_h, DEFAULT_H_ANGLE)
    set_servo(pwm_v, DEFAULT_V_ANGLE)

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()

    time.sleep(0.2)
    clock = time.clock()

    #可视化热待机死循环
    standby_active = True
    while standby_active:
        clock.tick()
        set_servo(pwm_h, DEFAULT_H_ANGLE)
        set_servo(pwm_v, DEFAULT_V_ANGLE)

        if laser_pin is not None:
            laser_pin.value(0)

        if trigger_pin.value() == 0:
            standby_active = False
            current_h_angle = DEFAULT_H_ANGLE
            current_v_angle = DEFAULT_V_ANGLE
            #触发瞬间立刻点亮激光
            if laser_pin is not None:
                laser_pin.value(1)
            break

        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img.draw_circle(TARGET_POINT[0], TARGET_POINT[1], 4, color=(255, 0, 255), thickness=1)
        img.draw_line(TARGET_POINT[0]-20, TARGET_POINT[1], TARGET_POINT[0]+20, TARGET_POINT[1], color=(255, 0, 255), thickness=1)
        img.draw_line(TARGET_POINT[0], TARGET_POINT[1]-20, TARGET_POINT[0], TARGET_POINT[1]+20, color=(255, 0, 255), thickness=1)

        if (ticks_ms() // 500) % 2 == 0:
            img.draw_string_advanced(80, 100, 45, "STANDBY MODE", color=(255, 255, 0))
            img.draw_string_advanced(40, 160, 20, "Camera Active | Laser Safely OFF", color=(0, 255, 255))

        img.compress_for_ide()
        Display.show_image(img, x=(800-320)//2, y=(480-240)//2)
        os.exitpoint()

    #正式主循环

    prev_min_corners = None
    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_binary = img.to_grayscale(copy=True)
        img_binary = img_binary.binary([black])
        img_binary.dilate(5)
        img_binary.erode(2)

        #始终保持激光常亮
        if laser_pin is not None:
            laser_pin.value(1)


        #高速扫掠防畸变，低速追踪严防杂物
        if pid_active:
            CURRENT_THRESH = 1100
            CURRENT_MAX_ANGLE = 25
            CURRENT_MIN_RATIO = 0.07
            CURRENT_MIN_AREA = TRACK_MIN_AREA
            MIN_ASPECT_RATIO = 0.5
            MAX_ASPECT_RATIO = 2.0
            mode_text = "Strict Track"
        else:
            CURRENT_THRESH = 350
            CURRENT_MAX_ANGLE = 75
            CURRENT_MIN_RATIO = 0.04
            CURRENT_MIN_AREA = SCAN_MIN_AREA
            MIN_ASPECT_RATIO = 0.1
            MAX_ASPECT_RATIO = 5.0
            mode_text = "Insane Sweep"

        rects = img_binary.find_rects(threshold=CURRENT_THRESH)

        min_rect = None
        min_area_val = float('inf')
        min_corners = None
        min_black_ratio = 0
        survivors = []

        if rects is not None:
            for rect in rects:
                corners = rect.corners()
                if len(corners) != 4: continue

                #1.面积过滤
                current_area = rect.w() * rect.h()
                if current_area < CURRENT_MIN_AREA:
                    continue

                #2.长宽比动态过滤
                aspect_ratio = rect.w() / rect.h() if rect.h() > 0 else 0
                if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                    continue

                #3.角度畸变动态过滤
                angles = []
                max_angle_error = 0
                for i in range(4):
                    p0, p1, p2 = corners[(i-1)%4], corners[i], corners[(i+1)%4]
                    angle_diff = vector_angle_diff((p0[0]-p1[0],p0[1]-p1[1]), (p2[0]-p1[0],p2[1]-p1[1]))
                    max_angle_error = max(max_angle_error, abs(angle_diff - 90))
                    angles.append(abs(angle_diff - 90))

                if max_angle_error > CURRENT_MAX_ANGLE: continue

                #4.黑占比过滤
                center = get_line_intersection([corners[0], corners[2]], [corners[1], corners[3]])
                cx, cy = int(center[0]), int(center[1])

                center_rect_x_start = max(0, cx - rect.w() // 4)
                center_rect_x_end = min(img.width() - 1, cx + rect.w() // 4)
                center_rect_y_start = max(0, cy - rect.h() // 4)
                center_rect_y_end = min(img.height() - 1, cy + rect.h() // 4)

                valid_pixels = 0
                total_pixels = 0
                step_size = 7
                for y in range(center_rect_y_start, center_rect_y_end, step_size):
                    for x in range(center_rect_x_start, center_rect_x_end, step_size):
                        pixel_value = img_binary.get_pixel(x, y)
                        if isinstance(pixel_value, tuple): pixel_value = pixel_value[0]
                        if pixel_value == 0: valid_pixels += 1
                        total_pixels += 1

                black_ratio = valid_pixels / total_pixels if total_pixels > 0 else 0.0

                if black_ratio < CURRENT_MIN_RATIO:
                    continue

                survivors.append((rect, corners, black_ratio, current_area))
                if current_area < min_area_val:
                    min_area_val, min_rect, min_corners = current_area, rect, corners

        if len(survivors) > 0:
            flag_detected = True
            lost_counter = 0
        else:
            lost_counter += 1
            if lost_counter > min_lost_frames: flag_detected = False

        if min_rect is not None:
            current_size_proxy = math.sqrt(min_area_val)
        else:
            current_size_proxy = math.sqrt(REF_BOX_WIDTH * REF_BOX_WIDTH)

        dynamic_offset_x = int(current_size_proxy * LASER_OFFSET_X_RATIO) + BASE_OFFSET_X
        dynamic_offset_y = int(current_size_proxy * LASER_OFFSET_Y_RATIO) + BASE_OFFSET_Y

        raw_laser_x = TARGET_POINT[0] + dynamic_offset_x
        raw_laser_y = TARGET_POINT[1] - dynamic_offset_y
        LASER_POINT_X = max(10, min(310, raw_laser_x))
        LASER_POINT_Y = max(10, min(230, raw_laser_y))

        if flag_detected:
            if min_corners is not None:
                img.draw_line(min_corners[0][0], min_corners[0][1], min_corners[1][0], min_corners[1][1], color=(0, 255, 0), thickness=2)
                img.draw_line(min_corners[1][0], min_corners[1][1], min_corners[2][0], min_corners[2][1], color=(0, 255, 0), thickness=2)
                img.draw_line(min_corners[2][0], min_corners[2][1], min_corners[3][0], min_corners[3][1], color=(0, 255, 0), thickness=2)
                img.draw_line(min_corners[3][0], min_corners[3][1], min_corners[0][0], min_corners[0][1], color=(0, 255, 0), thickness=2)

                # 绘制青色对角交叉线，标定靶心
                img.draw_line(min_corners[0][0], min_corners[0][1], min_corners[2][0], min_corners[2][1], color=(0, 255, 255), thickness=1)
                img.draw_line(min_corners[1][0], min_corners[1][1], min_corners[3][0], min_corners[3][1], color=(0, 255, 255), thickness=1)

                diagonal1, diagonal2 = [min_corners[0], min_corners[2]], [min_corners[1], min_corners[3]]
                center = get_line_intersection(diagonal1, diagonal2)
                center_x, center_y = float(center[0]), float(center[1])

                if not pid_active:
                    smooth_x, smooth_y = center_x, center_y
                    prev_raw_x, prev_raw_y = center_x, center_y
                    last_dx, last_dy, integral_x, integral_y = LASER_POINT_X - center_x, LASER_POINT_Y - center_y, 0, 0
                    pid_active = True

                smooth_x = (EMA_ALPHA * center_x) + ((1 - EMA_ALPHA) * smooth_x)
                smooth_y = (EMA_ALPHA * center_y) + ((1 - EMA_ALPHA) * smooth_y)
                vel_x, vel_y = center_x - prev_raw_x, center_y - prev_raw_y
                prev_raw_x, prev_raw_y = center_x, center_y


                pred_x, pred_y = smooth_x, smooth_y

                dx_center = LASER_POINT_X - pred_x
                dy_center = LASER_POINT_Y - pred_y

                if abs(dx_center) < DEADZONE:
                    dx_center = 0
                    integral_x = 0  #进入死区立刻清空积分，防抽搐
                if abs(dy_center) < DEADZONE:
                    dy_center = 0
                    integral_y = 0

                integral_x += dx_center
                integral_y += dy_center
                #=积分限幅防 Windup
                integral_x = max(-I_MAX/KI_X, min(I_MAX/KI_X, integral_x)) if KI_X != 0 else 0
                integral_y = max(-I_MAX/KI_Y, min(I_MAX/KI_Y, integral_y)) if KI_Y != 0 else 0

                delta_h = (KP_X * dx_center) + (KI_X * integral_x) + (KD_X * (dx_center - last_dx))
                delta_v = (KP_Y * dy_center) + (KI_Y * integral_y) + (KD_Y * (dy_center - last_dy))
                last_dx, last_dy = dx_center, dy_center

                current_h_angle += max(-MAX_TRACK_STEP, min(MAX_TRACK_STEP, delta_h))
                current_v_angle -= max(-MAX_TRACK_STEP, min(MAX_TRACK_STEP, delta_v))
        else:
            pid_active = False
            current_v_angle += 2.0 if (DEFAULT_V_ANGLE - current_v_angle) > 0 else -2.0 if abs(DEFAULT_V_ANGLE - current_v_angle) > 2.0 else 0
            current_h_angle += SCAN_SPEED_H * scan_direction
            if current_h_angle >= H_ANGLE_MAX: current_h_angle, scan_direction = H_ANGLE_MAX, -1
            if current_h_angle <= H_ANGLE_MIN: current_h_angle, scan_direction = H_ANGLE_MIN, 1

        current_h_angle = max(H_ANGLE_MIN, min(H_ANGLE_MAX, current_h_angle))
        current_v_angle = max(V_ANGLE_MIN, min(V_ANGLE_MAX, current_v_angle))
        set_servo(pwm_h, current_h_angle)
        set_servo(pwm_v, current_v_angle)

        img.draw_circle(TARGET_POINT[0], TARGET_POINT[1], 3, color=(255, 0, 255), thickness=1)
        img.draw_line(TARGET_POINT[0]-10, TARGET_POINT[1], TARGET_POINT[0]+10, TARGET_POINT[1], color=(255, 0, 255), thickness=1)
        img.draw_line(TARGET_POINT[0], TARGET_POINT[1]-10, TARGET_POINT[0], TARGET_POINT[1]+10, color=(255, 0, 255), thickness=1)

        #红色微十字准心
        lx, ly = int(LASER_POINT_X), int(LASER_POINT_Y)
        img.draw_circle(lx, ly, 2, color=(255, 0, 0), thickness=1, fill=True)
        img.draw_line(lx - 6, ly, lx + 6, ly, color=(255, 0, 0), thickness=1)
        img.draw_line(lx, ly - 6, lx, ly + 6, color=(255, 0, 0), thickness=1)

        img.draw_string_advanced(10, 10, 15, f"fps: {clock.fps():.1f} | Mode: {mode_text}", color=(255, 0, 0))
        img.draw_string_advanced(10, 50, 15, f"H_Ang: {current_h_angle:.1f} V_Ang: {current_v_angle:.1f}", color=(0, 255, 255))

        if pid_active:
            img.draw_string_advanced(10, 90, 15, f"Area: {min_area_val}", color=(0, 255, 0))
            img.draw_string_advanced(10, 110, 15, f"Offset X:{dynamic_offset_x} Y:{dynamic_offset_y}", color=(255, 100, 100))

            if raw_laser_y < 10 or raw_laser_x < 10 or raw_laser_x > 310:
                img.draw_string_advanced(10, 140, 15, "WARNING: CLAMPED!", color=(255, 0, 0))

        img.compress_for_ide()
        Display.show_image(img, x=(800-320)//2, y=(480-240)//2)

except Exception as e:
    print("发生异常:", e)
finally:
    try:
        if laser_pin is not None:
            laser_pin.value(0)
    except: pass

    try:
        if isinstance(sensor, Sensor): sensor.stop()
    except: pass
    try:
        Display.deinit()
    except: pass
    try:
        if pwm_h: pwm_h.deinit()
        if pwm_v: pwm_v.deinit()
    except: pass
    try:
        MediaManager.deinit()
    except: pass
    print("资源已安全释放。")
