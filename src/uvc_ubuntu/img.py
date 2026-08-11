import subprocess
import cv2
import numpy as np
import sys
import os

# --- 配置参数 ---
WIDTH = 384
HEIGHT = 288
FRAME_SIZE = WIDTH * HEIGHT * 2  # YUYV 每个像素 2 字节

def main():
    if not os.path.exists("./uvc_demo"):
        print("错误: 找不到 ./uvc_demo，请先执行 make 编译！")
        return

    # 注意：这里不要写 sudo，运行时用 sudo python3 img.py
    cmd = ["./uvc_demo"]

    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=sys.stderr)
    except Exception as e:
        print(f"启动失败: {e}")
        return

    print(f"\n>>> 窗口已打开 ({WIDTH}x{HEIGHT})")
    print(">>> 【调试操作说明】:")
    print("    [A / D] : 左右微调 (水平)")
    print("    [W / S] : 上下微调 (垂直)")
    print("    [ESC/q] : 退出")

    # 你当前调出来的偏移量
    shift_x = 0
    shift_y = 0

    while True:
        raw_data = process.stdout.read(FRAME_SIZE)

        if not raw_data or len(raw_data) != FRAME_SIZE:
            print("读取图像数据失败或长度不匹配")
            break

        try:
            raw = np.frombuffer(raw_data, dtype=np.uint8)

            # YUYV 原始数据：HEIGHT × WIDTH × 2
            yuv = raw.reshape((HEIGHT, WIDTH, 2))

            # 垂直滚动
            if shift_y != 0:
                yuv = np.roll(yuv, shift_y, axis=0)

            # 水平滚动
            if shift_x != 0:
                yuv = np.roll(yuv, shift_x, axis=1)

            # 计算对应 C 代码字节修正值
            total_byte_fix = (shift_y * WIDTH * 2) + (shift_x * 2)

            # 只取 Y 通道作为灰度热图
            gray = yuv[:, :, 0]

            # 增强显示对比度
            gray_show = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)

            # 转成 BGR，方便画彩色文字
            bgr = cv2.cvtColor(gray_show, cv2.COLOR_GRAY2BGR)

            # 放大显示
            display_img = cv2.resize(
                bgr,
                (WIDTH * 3, HEIGHT * 3),
                interpolation=cv2.INTER_NEAREST
            )

            info_text = f"X: {shift_x} | Y: {shift_y}"
            byte_text = f"Byte Offset Fix: {total_byte_fix}"

            cv2.putText(display_img, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.putText(display_img, byte_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.putText(display_img, "Press W/S/A/D to adjust",
                        (10, HEIGHT * 3 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow("Thermal Gray Calibration Mode", display_img)

        except Exception as e:
            print(f"图像解析错误: {e}")
            continue

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            break
        elif key == ord('a'):
            shift_x -= 1
            print(f"shift_x={shift_x}, shift_y={shift_y}")
        elif key == ord('d'):
            shift_x += 1
            print(f"shift_x={shift_x}, shift_y={shift_y}")
        elif key == ord('w'):
            shift_y -= 1
            print(f"shift_x={shift_x}, shift_y={shift_y}")
        elif key == ord('s'):
            shift_y += 1
            print(f"shift_x={shift_x}, shift_y={shift_y}")

    process.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
