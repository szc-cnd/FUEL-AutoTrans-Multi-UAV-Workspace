#!/usr/bin/env python3
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

import cv2
import numpy as np


WIDTH = 384
HEIGHT = 288
FRAME_SIZE = WIDTH * HEIGHT * 2


def _is_executable_file(path):
    return bool(path) and os.path.isfile(path) and os.access(path, os.X_OK)


def _find_ros_uvc_demo():
    try:
        from roslib.packages import find_node
    except (ImportError, ModuleNotFoundError):
        return None

    try:
        matches = find_node("uvc_ubuntu", "uvc_demo") or []
    except Exception:
        return None

    if isinstance(matches, str):
        matches = [matches]
    for match in matches:
        if _is_executable_file(match):
            return os.path.abspath(match)
    return None


def resolve_uvc_demo_path(uvc_demo_path):
    # A caller-provided path with an explicit directory is authoritative. Keep
    # it even before checking execute permission so start_uvc_demo can report a
    # useful permission error instead of silently selecting another binary.
    if uvc_demo_path and os.path.dirname(os.path.expanduser(uvc_demo_path)):
        explicit_path = os.path.abspath(os.path.expanduser(uvc_demo_path))
        if os.path.isfile(explicit_path):
            return explicit_path
    if _is_executable_file(uvc_demo_path):
        return os.path.abspath(uvc_demo_path)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    fallback_path = os.path.join(script_dir, uvc_demo_path)
    if _is_executable_file(fallback_path):
        return os.path.abspath(fallback_path)

    ros_path = _find_ros_uvc_demo()
    if ros_path:
        return ros_path

    path_match = shutil.which(os.path.basename(uvc_demo_path))
    if path_match:
        return path_match

    return uvc_demo_path


def build_uvc_command(uvc_demo_path):
    cmd = [uvc_demo_path]
    if shutil.which("stdbuf"):
        cmd = ["stdbuf", "-oL", "-eL"] + cmd
    return cmd


def drain_startup_logs(stream, timeout=8.0):
    """Drain text logs printed by uvc_demo before raw frame bytes start."""
    if stream is None:
        return

    try:
        import select
    except ImportError:
        return

    fd = stream.fileno()
    deadline = time.time() + timeout

    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.2)
        if not ready:
            continue

        line = stream.readline()
        if not line:
            return

        text = line.decode("utf-8", errors="replace").rstrip()
        if text:
            print(f"[uvc_demo] {text}", file=sys.stderr)

        if b"hik sensor: w =" in line:
            return


def start_uvc_demo(uvc_demo_path="./uvc_demo", uvc_offset_fix=None):
    uvc_demo_path = resolve_uvc_demo_path(uvc_demo_path)
    if not _is_executable_file(uvc_demo_path):
        raise FileNotFoundError(
            f"Cannot find executable {uvc_demo_path}. Build uvc_demo or fix its execute permission."
        )

    cmd = build_uvc_command(uvc_demo_path)
    cwd = os.path.dirname(os.path.abspath(uvc_demo_path))
    env = os.environ.copy()
    if uvc_offset_fix is not None:
        env["HIK_YUV_OFFSET_FIX"] = str(uvc_offset_fix)
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
        cwd=cwd,
        env=env,
        bufsize=0,
        preexec_fn=os.setsid if hasattr(os, "setsid") else None,
    )
    drain_startup_logs(process.stdout)
    return process


def stop_process(process, timeout=2.0):
    if process is None:
        return
    if process.poll() is not None:
        return

    try:
        if hasattr(os, "killpg"):
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        else:
            process.terminate()
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        if hasattr(os, "killpg"):
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        else:
            process.kill()
        process.wait(timeout=timeout)
    except Exception as exc:
        print(f"Warning: failed to stop uvc_demo cleanly: {exc}", file=sys.stderr)


def read_frame(process, frame_size):
    if process.stdout is None:
        print("Error: uvc_demo stdout is not available", file=sys.stderr)
        return None

    data = bytearray()
    while len(data) < frame_size:
        chunk = process.stdout.read(frame_size - len(data))
        if not chunk:
            rc = process.poll()
            if rc is None:
                print(
                    f"Error: short frame read ({len(data)}/{frame_size}), retrying",
                    file=sys.stderr,
                )
                return None
            print(
                f"Error: uvc_demo exited with code {rc}, short frame read "
                f"({len(data)}/{frame_size})",
                file=sys.stderr,
            )
            return None
        data.extend(chunk)

    return bytes(data)


def yuyv_to_gray(raw_data, width, height):
    expected = width * height * 2
    if raw_data is None or len(raw_data) != expected:
        raise ValueError(f"Expected {expected} bytes, got {0 if raw_data is None else len(raw_data)}")
    raw = np.frombuffer(raw_data, dtype=np.uint8)
    yuv = raw.reshape((height, width, 2))
    return yuv[:, :, 0].copy()


def shift_image_no_wrap(image, shift_x=0, shift_y=0, fill_value=0):
    h, w = image.shape[:2]
    shifted = np.full_like(image, fill_value)

    src_x0 = max(0, -shift_x)
    src_x1 = min(w, w - shift_x)
    dst_x0 = max(0, shift_x)
    dst_x1 = min(w, w + shift_x)

    src_y0 = max(0, -shift_y)
    src_y1 = min(h, h - shift_y)
    dst_y0 = max(0, shift_y)
    dst_y1 = min(h, h + shift_y)

    if src_x0 >= src_x1 or src_y0 >= src_y1:
        return shifted

    shifted[dst_y0:dst_y1, dst_x0:dst_x1] = image[src_y0:src_y1, src_x0:src_x1]
    return shifted


def detect_hotspot(
    gray,
    roi_margin_x=30,
    roi_margin_y=20,
    k=2.0,
    min_area=20,
    max_area=5000,
):
    h, w = gray.shape[:2]
    x0 = max(0, min(roi_margin_x, w - 1))
    y0 = max(0, min(roi_margin_y, h - 1))
    x1 = max(x0 + 1, w - x0)
    y1 = max(y0 + 1, h - y0)
    roi = gray[y0:y1, x0:x1]

    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    mean, std = cv2.meanStdDev(blur)
    threshold_value = float(mean[0][0] + k * std[0][0])
    threshold_value = max(0.0, min(255.0, threshold_value))

    mask = (blur > threshold_value).astype(np.uint8) * 255
    if mask.shape[0] >= 5 and mask.shape[1] >= 5:
        mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < min_area or area > max_area:
            continue

        cnt_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
        mean_gray = float(cv2.mean(blur, mask=cnt_mask)[0])

        # Prefer the brightest valid blob; use area as the tie breaker.
        score = (mean_gray, area)
        if best is None or score > best["score"]:
            moments = cv2.moments(cnt)
            if moments["m00"] == 0:
                continue
            cx_roi = int(moments["m10"] / moments["m00"])
            cy_roi = int(moments["m01"] / moments["m00"])
            bx, by, bw, bh = cv2.boundingRect(cnt)
            best = {
                "score": score,
                "cx": cx_roi + x0,
                "cy": cy_roi + y0,
                "bbox": (bx + x0, by + y0, bw, bh),
                "area": area,
                "mean_gray": mean_gray,
            }

    result = {
        "detected": best is not None,
        "cx": -1,
        "cy": -1,
        "bbox": None,
        "area": 0.0,
        "mean_gray": 0.0,
        "threshold": threshold_value,
        "roi_rect": (x0, y0, x1 - x0, y1 - y0),
        "mask": mask,
    }

    if best is not None:
        result.update(
            {
                "cx": best["cx"],
                "cy": best["cy"],
                "bbox": best["bbox"],
                "area": best["area"],
                "mean_gray": best["mean_gray"],
            }
        )

    return result


def draw_debug(gray, detection, stable_detected=None, stable_count=None, stable_window=None):
    debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # When called by thermal_ros_node, stable_detected is the same detector
    # gate published in target_candidate_status.  The standalone detector
    # keeps the historical behavior by treating a raw detection as stable.
    if stable_detected is None:
        stable_detected = bool(detection["detected"])
    if stable_count is None:
        stable_count = 1 if detection["detected"] else 0
    if stable_window is None:
        stable_window = 1

    rx, ry, rw, rh = detection["roi_rect"]
    cv2.rectangle(debug, (rx, ry), (rx + rw - 1, ry + rh - 1), (255, 160, 0), 1)

    if detection["detected"]:
        x, y, w, h = detection["bbox"]
        cx, cy = detection["cx"], detection["cy"]
        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.drawMarker(
            debug,
            (cx, cy),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=12,
            thickness=2,
        )
        status = (
            f"candidate=True stable={bool(stable_detected)} "
            f"{int(stable_count)}/{int(stable_window)} pixel=({cx},{cy})"
        )
    else:
        status = "candidate=False stable=False pixel=(-1,-1)"

    cv2.putText(debug, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    cv2.putText(
        debug,
        f"thr={detection['threshold']:.1f} area={detection['area']:.1f}",
        (8, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        1,
    )
    return debug


def resize_for_display(image, scale):
    if scale == 1.0:
        return image
    return cv2.resize(
        image,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_NEAREST,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Competition thermal hotspot detector")
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    parser.add_argument("--shift-x", type=int, default=0)
    parser.add_argument("--shift-y", type=int, default=0)
    parser.add_argument("--roi-margin-x", type=int, default=30)
    parser.add_argument("--roi-margin-y", type=int, default=20)
    parser.add_argument("--k", type=float, default=2.0)
    parser.add_argument("--min-area", type=float, default=20)
    parser.add_argument("--max-area", type=float, default=5000)
    parser.add_argument("--uvc-demo-path", default="./uvc_demo")
    parser.add_argument(
        "--uvc-offset-fix",
        type=int,
        default=None,
        help="Set HIK_YUV_OFFSET_FIX for uvc_demo; requires rebuilding uvc_demo.",
    )
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--save", action="store_true")
    parser.add_argument("--save-dir", default="thermal_detect_output")
    parser.add_argument("--save-every", type=int, default=10)
    parser.add_argument("--display-scale", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    frame_size = args.width * args.height * 2

    process = None
    frame_id = 0
    saved_count = 0

    if args.save:
        os.makedirs(args.save_dir, exist_ok=True)

    try:
        process = start_uvc_demo(args.uvc_demo_path, args.uvc_offset_fix)
        print("Thermal detector started. Press q/ESC in the image window to exit.")

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
            detection = detect_hotspot(
                gray,
                roi_margin_x=args.roi_margin_x,
                roi_margin_y=args.roi_margin_y,
                k=args.k,
                min_area=args.min_area,
                max_area=args.max_area,
            )
            debug = draw_debug(gray, detection)

            print(
                f"detected={detection['detected']} "
                f"cx={detection['cx']} cy={detection['cy']} "
                f"threshold={detection['threshold']:.1f} area={detection['area']:.1f}",
                flush=True,
            )

            if args.save and frame_id % max(1, args.save_every) == 0:
                out_path = os.path.join(args.save_dir, f"thermal_{saved_count:06d}.jpg")
                cv2.imwrite(out_path, debug)
                saved_count += 1

            if not args.no_display:
                show = resize_for_display(debug, args.display_scale)
                cv2.imshow("Thermal Hotspot Detector", show)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break

            frame_id += 1

    except KeyboardInterrupt:
        print("Interrupted, exiting.")
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
    finally:
        stop_process(process)
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
