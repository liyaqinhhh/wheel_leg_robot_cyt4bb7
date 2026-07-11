#!/usr/bin/env python3

# -*- coding: utf-8 -*-

"""

=============================================================================

 逐飞科技 (Seekfree) MT9V03X 无线串口图传 —— 上位机实时可视化脚本

=============================================================================



 依赖安装 (请在终端中执行):

   pip install pyserial opencv-python numpy



 功能:

   1. 通过无线串口接收 CYT4BB7 小车发来的图像数据

   2. 严格按照逐飞助手图传协议解析帧头、类型、尺寸、图像数据

   3. 实时显示 "原始灰度图像" 和 "二值化图像" 双窗口

   4. 鲁棒的错帧/半帧/断连处理, 自动重新同步帧头



 协议参考源:

   下位机工程: Example/Motherboard_Demo/E8_camera/

              E8_02_mt9v03x_wireless_uart_seekfree_assistant_demo

   协议定义:   libraries/zf_components/seekfree_assistant.h

              libraries/zf_components/seekfree_assistant.c



 协议帧格式 (图像帧):

   ┌──────┬──────┬─────────────┬────────┬──────────────┬───────────────┐

   │ 0xAA │ 0x02 │ camera_type │ length │ image_width  │ image_height  │

   │  1B  │  1B  │     1B      │   1B   │  2B (LE)     │  2B (LE)      │

   └──────┴──────┴─────────────┴────────┴──────────────┴───────────────┘

   头部共 8 字节, 后跟图像数据 (灰度: W*H 字节, 二值: W*H/8 字节)



 camera_type 编码:

   bits[7:5] = 图像类型 (1=二值OV7725, 2=灰度MT9V03X, 3=RGB565)

   bit[4]    = 是否有图像数据 (0=有图像, 1=仅边界)

   bits[3:0] = 边界数量



 协议帧格式 (边界打点帧):

   ┌──────┬──────┬──────────┬────────┬──────────┬────────────┬─────────┐

   │ 0xAA │ 0x03 │ dot_type │ length │ dot_num  │ valid_flag │ reserve │

   │  1B   │  1B  │   1B     │   1B   │ 2B (LE)  │    1B      │   1B    │

   └──────┴──────┴──────────┴────────┴──────────┴────────────┴─────────┘

   头部共 10 字节, 后跟边界坐标数组



=============================================================================

"""



import struct

import time

import numpy as np



# ========================== 全局 pip install 命令 ==========================

# pip install pyserial opencv-python numpy



import serial

import cv2





# =========================================================================

#  全局宏定义配置区 (集中在这里修改)

# =========================================================================



# --- 串口配置 ---

COM_PORT        = "COM12"       # 无线串口号

BAUD_RATE       = 115200        # 波特率 (逐飞无线串口默认 115200)

# BAUD_RATE     = 460800        # 可改为 460800

# BAUD_RATE     = 2000000       # 可改为 2000000

SERIAL_TIMEOUT  = 0.05          # 串口读取超时(秒), 不宜过大以保持低延迟



# --- 图像显示窗口配置 ---

WINDOW_GRAY     = "Gray Image (MT9V03X)"       # 灰度图窗口名

WINDOW_BINARY   = "Binary Image (Otsu)"         # 二值图窗口名

DISPLAY_WIDTH   = 376                           # 显示缩放宽度 (原始 188*2)

DISPLAY_HEIGHT  = 240                           # 显示缩放高度 (原始 120*2)



# --- 协议常量 (与逐飞 seekfree_assistant.h 对齐, 勿修改) ---

FRAME_HEAD          = 0xAA  # 单片机→上位机 帧头

CAMERA_FUNCTION     = 0x02  # 图像帧功能字

DOT_FUNCTION        = 0x03  # 边界打点功能字

OSCILLOSCOPE_FUNC   = 0x10  # 虚拟示波器功能字 (跳过不处理)



HEADER_SIZE         = 8     # 图像帧头字节数

DOT_HEADER_SIZE     = 10    # 边界帧头字节数



# 图像类型枚举 (对应 seekfree_assistant_image_type_enum)

IMAGE_TYPE_BINARY   = 1     # OV7725 二值图  → payload = W*H/8

IMAGE_TYPE_GRAY     = 2     # MT9V03X 灰度图 → payload = W*H

IMAGE_TYPE_RGB565   = 3     # SCC8660 RGB565  → payload = W*H*2



# --- 帧头同步超时 ---

HEAD_SYNC_TIMEOUT   = 2.0   # 超过此秒数未收到有效帧头, 清空缓冲区重新同步



# =========================================================================

#  全局状态管理器 —— smart_car_see

# =========================================================================



class SmartCarSee:

    """统筹管理图像缓存、串口状态、协议解析状态机"""



    def __init__(self):

        # --- 串口 ---

        self.ser: serial.Serial | None = None



        # --- 接收原始字节缓冲区 ---

        self.raw_buffer = bytearray()



        # --- 最新图像缓存 ---

        self.gray_image: np.ndarray | None = None      # 灰度图 (188x120)

        self.binary_image: np.ndarray | None = None     # 二值图 (188x120)



        # --- 更新标志(线程安全用, 单线程无需锁) ---

        self.gray_updated   = False

        self.binary_updated = False



        # --- 运行控制 ---

        self.running = True



        # --- 统计 ---

        self.frame_count  = 0

        self.error_count  = 0

        self.last_good_time = time.time()   # 最后一次收到有效帧的时间



    def reset(self):

        """重置接收状态"""

        self.raw_buffer.clear()





# 全局单例

smart_car_see = SmartCarSee()





# =========================================================================

#  协议解析 — 字节级状态机

# =========================================================================



class FrameParser:

    """

    逐飞图传协议字节流解析状态机



    状态转移:

      WAIT_HEAD  → 找到 0xAA → WAIT_FUNC 或 WAIT_HEAD (功能字不匹配则跳回)

      WAIT_FUNC  → 根据功能字 → WAIT_HEADER(图像) / WAIT_DOT_HEADER(边界) / WAIT_HEAD(跳过示波器)

      WAIT_HEADER→ 凑齐8字节  → 校验尺寸合理性 → WAIT_IMAGE 或 WAIT_HEAD(不合理则丢弃)

      WAIT_IMAGE → 凑齐payload→ 解码并更新图像缓存 → WAIT_HEAD

    """



    # 状态枚举

    (WAIT_HEAD, WAIT_FUNC, WAIT_HEADER,

     WAIT_IMAGE, WAIT_DOT_HEADER, WAIT_DOT_DATA) = range(6)



    def __init__(self, see: SmartCarSee):

        self.see = see

        self.state = self.WAIT_HEAD



        # 当前帧解析中的临时变量

        self.cur_func        = 0

        self.cur_camera_type = 0

        self.cur_image_type  = 0

        self.cur_width       = 0

        self.cur_height      = 0

        self.cur_payload_len = 0

        self.cur_payload_buf = bytearray()



        # 已丢弃的不匹配0xAA计数

        self.skipped_bytes = 0



    def reset_state(self):

        """丢弃当前帧, 回到等待帧头状态"""

        self.state = self.WAIT_HEAD

        self.cur_payload_buf.clear()

        self.cur_payload_len = 0



    def feed(self, new_data: bytes):

        """

        喂入新接收的字节数据, 内部驱动状态机

        返回: 成功解析的图像类型 (IMAGE_TYPE_GRAY/IMAGE_TYPE_BINARY), 或 None

        """

        buf = self.see.raw_buffer

        buf.extend(new_data)



        result_type = None



        # 限制缓冲区最大大小, 防止异常积累

        MAX_BUF = 256 * 1024  # 256KB

        if len(buf) > MAX_BUF:

            # 缓冲区过大, 清空并从末尾可能包含帧头的位置开始

            print(f"[WARN] 缓冲区过大({len(buf)}), 清空重同步")

            buf.clear()

            self.reset_state()

            return None



        while True:

            if self.state == self.WAIT_HEAD:

                # ---- 寻找帧头 0xAA ----

                if len(buf) < 1:

                    break

                b = buf[0]

                if b == FRAME_HEAD:

                    del buf[0]

                    self.state = self.WAIT_FUNC

                else:

                    del buf[0]

                    self.skipped_bytes += 1



            elif self.state == self.WAIT_FUNC:

                # ---- 读取功能字 ----

                if len(buf) < 1:

                    break

                func = buf[0]

                if func == CAMERA_FUNCTION:

                    del buf[0]

                    self.cur_func = func

                    self.state = self.WAIT_HEADER

                elif func == DOT_FUNCTION:

                    del buf[0]

                    self.cur_func = func

                    self.state = self.WAIT_DOT_HEADER

                elif func == OSCILLOSCOPE_FUNC:

                    # 示波器帧: 跳过, 不做处理 (格式不同, 需按结构体跳过)

                    # 示波器帧头 = 4 + N*4 字节, 无法简单跳过, 回到 WAIT_HEAD

                    del buf[0]

                    self.state = self.WAIT_HEAD

                else:

                    # 不是任何已知功能字, 丢弃这个0xAA, 重新找帧头

                    del buf[0]

                    self.state = self.WAIT_HEAD



            elif self.state == self.WAIT_HEADER:

                # ---- 接收图像帧剩余 6 字节头部 (共8字节, 已读head+func) ----

                if len(buf) < 6:

                    break



                camera_type, length, width, height = struct.unpack_from(

                    "<BBHH", buf, 0)



                # 帧头长度校验: 必须为 HEADER_SIZE (8)

                if length != HEADER_SIZE:

                    # 长度不对, 可能是误同步, 丢弃第一个字节后重试

                    del buf[0]

                    self.state = self.WAIT_HEAD

                    self.see.error_count += 1

                    continue



                # 图像类型提取: bits[7:5]

                img_type = (camera_type >> 5) & 0x07



                # 尺寸合理性校验

                if not (1 <= width <= 800 and 1 <= height <= 600):

                    del buf[0]

                    self.state = self.WAIT_HEAD

                    self.see.error_count += 1

                    continue



                # 计算 payload 大小

                if img_type == IMAGE_TYPE_BINARY:

                    payload_len = width * height // 8

                elif img_type == IMAGE_TYPE_GRAY:

                    payload_len = width * height

                elif img_type == IMAGE_TYPE_RGB565:

                    payload_len = width * height * 2

                else:

                    del buf[0]

                    self.state = self.WAIT_HEAD

                    self.see.error_count += 1

                    continue



                # 头部校验通过, 提交

                del buf[0:6]

                self.cur_camera_type = camera_type

                self.cur_image_type  = img_type

                self.cur_width       = width

                self.cur_height      = height

                self.cur_payload_len = payload_len

                self.cur_payload_buf.clear()

                self.state = self.WAIT_IMAGE



            elif self.state == self.WAIT_IMAGE:

                # ---- 接收图像 payload ----

                if len(buf) < self.cur_payload_len:

                    # 数据不够, 等待下次 feed

                    break



                # 提取 payload

                payload = bytes(buf[:self.cur_payload_len])

                del buf[:self.cur_payload_len]



                # 解码图像

                result_type = self._decode_image(payload)



                # 回到等待帧头

                self.state = self.WAIT_HEAD

                # 一帧解析完成, 返回给调用者



                # 如果有结果, 立即返回 (一帧一帧处理)

                if result_type is not None:

                    return result_type



            elif self.state == self.WAIT_DOT_HEADER:

                # ---- 接收边界打点帧剩余 8 字节头部 (共10字节, 已读head+func) ----

                if len(buf) < 8:

                    break



                dot_type, length, dot_num, valid_flag, reserve = struct.unpack_from(

                    "<BBHBB", buf, 0)



                if length != DOT_HEADER_SIZE:

                    del buf[0]

                    self.state = self.WAIT_HEAD

                    continue



                # 判断坐标是8位还是16位

                is_16bit = (dot_type >> 5) & 0x01

                boundary_num = dot_type & 0x0F

                dot_bytes = dot_num * (2 if is_16bit else 1)

                total_dot_bytes = 0



                # 计算需要跳过的边界数据量

                # valid_flag 的 bit[0:2] 指示哪几条边界存在

                boundary_type = (dot_type >> 6) & 0x03

                if boundary_type == 0:  # X_BOUNDARY (只有X)

                    total_dot_bytes = bin(valid_flag & 0x07).count('1') * dot_bytes

                elif boundary_type == 1:  # Y_BOUNDARY (只有Y)

                    total_dot_bytes = bin(valid_flag & 0x07).count('1') * dot_bytes

                elif boundary_type == 2:  # XY_BOUNDARY (有X和Y)

                    total_dot_bytes = bin(valid_flag & 0x07).count('1') * dot_bytes * 2



                del buf[0:8]

                self.cur_payload_len = total_dot_bytes

                self.cur_payload_buf.clear()

                self.state = self.WAIT_DOT_DATA



            elif self.state == self.WAIT_DOT_DATA:

                # ---- 接收边界数据 (直接跳过, 不处理) ----

                if len(buf) < self.cur_payload_len:

                    break

                del buf[:self.cur_payload_len]

                self.state = self.WAIT_HEAD



        # 超时检测: 长时间未收到有效帧, 清空缓冲区

        if self.see.last_good_time > 0:

            if time.time() - self.see.last_good_time > HEAD_SYNC_TIMEOUT:

                if len(buf) > 0:

                    print(f"[WARN] 帧同步超时, 清空 {len(buf)} 字节重同步")

                    buf.clear()

                    self.reset_state()

                    self.see.last_good_time = time.time()



        return result_type



    def _decode_image(self, payload: bytes):

        """将 payload 解码为 OpenCV 图像并存入 smart_car_see"""

        w, h = self.cur_width, self.cur_height



        try:

            if self.cur_image_type == IMAGE_TYPE_GRAY:

                # 灰度图: 188×120 = 22560 字节

                expected = w * h

                if len(payload) != expected:

                    print(f"[ERR] 灰度图像素尺寸不匹配: got {len(payload)}, expect {expected}")

                    self.see.error_count += 1

                    return None



                img = np.frombuffer(payload, dtype=np.uint8).reshape((h, w))

                self.see.gray_image = img.copy()

                self.see.gray_updated = True

                self.see.frame_count += 1

                self.see.last_good_time = time.time()

                return IMAGE_TYPE_GRAY



            elif self.cur_image_type == IMAGE_TYPE_BINARY:

                # 二值图: 188×120/8 = 2820 字节, 每 bit = 1 像素

                expected = w * h // 8

                if len(payload) != expected:

                    print(f"[ERR] 二值图像素尺寸不匹配: got {len(payload)}, expect {expected}")

                    self.see.error_count += 1

                    return None



                # 逐 bit 展开为 0/255

                img = np.zeros((h, w), dtype=np.uint8)

                byte_idx = 0

                for row in range(h):

                    for col in range(0, w, 8):

                        byte_val = payload[byte_idx]

                        byte_idx += 1

                        for bit in range(8):

                            if col + bit < w:

                                if byte_val & (0x80 >> bit):

                                    img[row, col + bit] = 255

                self.see.binary_image = img

                self.see.binary_updated = True

                self.see.frame_count += 1

                self.see.last_good_time = time.time()

                return IMAGE_TYPE_BINARY



            elif self.cur_image_type == IMAGE_TYPE_RGB565:

                # RGB565: 188×120×2 = 45120 字节 → 转灰度显示

                expected = w * h * 2

                if len(payload) != expected:

                    print(f"[ERR] RGB565 像素尺寸不匹配: got {len(payload)}, expect {expected}")

                    self.see.error_count += 1

                    return None



                img_rgb565 = np.frombuffer(payload, dtype=np.uint16).reshape((h, w))

                # RGB565 → 灰度近似: (R*76 + G*150 + B*29) >> 8 的简化版

                r = ((img_rgb565 >> 11) & 0x1F) * 255 // 31

                g = ((img_rgb565 >> 5)  & 0x3F) * 255 // 63

                b = (img_rgb565        & 0x1F) * 255 // 31

                img_gray = (r.astype(np.float32) * 0.299 +

                            g.astype(np.float32) * 0.587 +

                            b.astype(np.float32) * 0.114).astype(np.uint8)

                self.see.gray_image = img_gray

                self.see.gray_updated = True

                self.see.frame_count += 1

                self.see.last_good_time = time.time()

                return IMAGE_TYPE_GRAY  # 按灰度显示



        except Exception as e:

            print(f"[ERR] 图像解码异常: {e}")

            self.see.error_count += 1

            return None



        return None





# =========================================================================

#  串口初始化

# =========================================================================



def serial_init() -> serial.Serial | None:

    """初始化串口连接"""

    print(f"[INFO] 尝试连接 {COM_PORT} @ {BAUD_RATE} bps ...")

    try:

        ser = serial.Serial(

            port=COM_PORT,

            baudrate=BAUD_RATE,

            bytesize=serial.EIGHTBITS,

            parity=serial.PARITY_NONE,

            stopbits=serial.STOPBITS_ONE,

            timeout=SERIAL_TIMEOUT,

        )

        print(f"[INFO] 串口 {COM_PORT} 打开成功")

        return ser

    except serial.SerialException as e:

        print(f"[ERR] 无法打开串口 {COM_PORT}: {e}")

        return None

    except Exception as e:

        print(f"[ERR] 串口初始化异常: {e}")

        return None





# =========================================================================

#  OpenCV 显示

# =========================================================================



def create_windows():

    """创建两个固定位置的显示窗口"""

    cv2.namedWindow(WINDOW_GRAY, cv2.WINDOW_NORMAL)

    cv2.namedWindow(WINDOW_BINARY, cv2.WINDOW_NORMAL)

    cv2.resizeWindow(WINDOW_GRAY, DISPLAY_WIDTH, DISPLAY_HEIGHT)

    cv2.resizeWindow(WINDOW_BINARY, DISPLAY_WIDTH, DISPLAY_HEIGHT)

    # 将窗口移动到屏幕左侧 (灰度) 和右侧 (二值)

    cv2.moveWindow(WINDOW_GRAY, 50, 50)

    cv2.moveWindow(WINDOW_BINARY, 50 + DISPLAY_WIDTH + 20, 50)





def update_display(see: SmartCarSee):

    """刷新双窗口显示"""

    gray_displayed = False

    binary_displayed = False



    if see.gray_image is not None and see.gray_updated:

        # 缩放显示

        disp = cv2.resize(see.gray_image, (DISPLAY_WIDTH, DISPLAY_HEIGHT),

                          interpolation=cv2.INTER_NEAREST)

        # 转为 BGR 彩色 (便于叠加文字)

        if len(disp.shape) == 2:

            disp = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)

        cv2.putText(disp, f"Gray  {see.gray_image.shape[1]}x{see.gray_image.shape[0]}  Frames:{see.frame_count}",

                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        cv2.imshow(WINDOW_GRAY, disp)

        see.gray_updated = False

        gray_displayed = True



    if see.binary_image is not None and see.binary_updated:

        disp = cv2.resize(see.binary_image, (DISPLAY_WIDTH, DISPLAY_HEIGHT),

                          interpolation=cv2.INTER_NEAREST)

        if len(disp.shape) == 2:

            disp = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)

        cv2.putText(disp, f"Binary  {see.binary_image.shape[1]}x{see.binary_image.shape[0]}  Frames:{see.frame_count}",

                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        cv2.imshow(WINDOW_BINARY, disp)

        see.binary_updated = False

        binary_displayed = True



    # 如果长时间没有图像, 显示等待提示

    if not gray_displayed and see.gray_image is None:

        blank = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

        cv2.putText(blank, "Waiting for gray image...",

                    (20, DISPLAY_HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX,

                    0.7, (255, 255, 255), 2)

        cv2.imshow(WINDOW_GRAY, blank)



    if not binary_displayed and see.binary_image is None:

        blank = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

        cv2.putText(blank, "Waiting for binary image...",

                    (20, DISPLAY_HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX,

                    0.7, (255, 255, 255), 2)

        cv2.imshow(WINDOW_BINARY, blank)





# =========================================================================

#  主循环

# =========================================================================



def main():

    print("=" * 60)

    print("  逐飞 MT9V03X 无线图传 - 上位机实时可视化")

    print(f"  串口: {COM_PORT}  波特率: {BAUD_RATE}")

    print("=" * 60)



    see = smart_car_see



    # 初始化串口

    ser = serial_init()

    if ser is None:

        print("[FATAL] 串口初始化失败, 退出")

        return

    see.ser = ser



    # 创建解析器

    parser = FrameParser(see)



    # 创建显示窗口

    create_windows()



    print("[INFO] 开始接收数据, 按 'q' 键退出, 按 'r' 键重置统计...")

    print()



    try:

        while see.running:

            # --- 读取串口数据 ---

            try:

                data = ser.read(ser.in_waiting or 1)

            except serial.SerialException as e:

                print(f"[ERR] 串口读取错误: {e}")

                time.sleep(0.5)

                continue



            if data:

                # 喂入解析器

                result = parser.feed(data)



                # 帧解析成功时的日志

                if result is not None:

                    if result == IMAGE_TYPE_GRAY:

                        if see.frame_count % 10 == 0:  # 每10帧打印一次

                            print(f"  [OK] 灰度帧 #{see.frame_count}  {parser.cur_width}x{parser.cur_height}")

                    elif result == IMAGE_TYPE_BINARY:

                        if see.frame_count % 10 == 0:

                            print(f"  [OK] 二值帧 #{see.frame_count}  {parser.cur_width}x{parser.cur_height}")



            # --- 刷新显示 ---

            update_display(see)



            # --- 键盘响应 (1ms 轮询) ---

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):

                print("[INFO] 用户按下 'q', 退出")

                see.running = False

                break

            elif key == ord('r'):

                see.frame_count = 0

                see.error_count = 0

                parser.skipped_bytes = 0

                print("[INFO] 统计已重置")



    except KeyboardInterrupt:

        print("\n[INFO] Ctrl+C 中断")

    finally:

        print(f"\n[INFO] 统计: 有效帧={see.frame_count}, 错误帧={see.error_count}")

        cv2.destroyAllWindows()

        if ser and ser.is_open:

            ser.close()

            print("[INFO] 串口已关闭")





if __name__ == "__main__":

    main()