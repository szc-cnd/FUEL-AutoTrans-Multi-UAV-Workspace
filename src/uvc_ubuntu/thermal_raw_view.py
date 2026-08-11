#!/usr/bin/env python3
import argparse
import os
import sys

import cv2
import rospkg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
try:
    PACKAGE_DIR = rospkg.RosPack().get_path("uvc_ubuntu")
except rospkg.ResourceNotFound:
    PACKAGE_DIR = SCRIPT_DIR
if not sys.path or sys.path[0] != PACKAGE_DIR:
    sys.path.insert(0, PACKAGE_DIR)

from thermal_detect import (
    HEIGHT,
    WIDTH,
    read_frame,
    resize_for_display,
    shift_image_no_wrap,
    start_uvc_demo,
    stop_process,
    yuyv_to_gray,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Raw thermal grayscale viewer")
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    parser.add_argument("--shift-x", type=int, default=0)
    parser.add_argument("--shift-y", type=int, default=0)
    parser.add_argument("--uvc-demo-path", default="./uvc_demo")
    parser.add_argument(
        "--uvc-offset-fix",
        type=int,
        default=None,
        help="Set HIK_YUV_OFFSET_FIX for uvc_demo; requires rebuilt uvc_demo.",
    )
    parser.add_argument("--display-scale", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    frame_size = args.width * args.height * 2
    process = None

    try:
        process = start_uvc_demo(args.uvc_demo_path, args.uvc_offset_fix)
        print("Raw thermal viewer started. Press q/ESC to exit.")

        while True:
            raw_data = read_frame(process, frame_size)
            if raw_data is None:
                if process.poll() is not None:
                    break
                continue

            try:
                gray = yuyv_to_gray(raw_data, args.width, args.height)
            except ValueError as exc:
                print(f"Frame parse error: {exc}", file=sys.stderr)
                continue

            gray = shift_image_no_wrap(gray, args.shift_x, args.shift_y, fill_value=0)
            show = resize_for_display(gray, args.display_scale)
            cv2.imshow("Raw Thermal Gray", show)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

    except KeyboardInterrupt:
        print("Interrupted, exiting.")
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
    finally:
        stop_process(process)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
