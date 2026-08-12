#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import threading
import time

from target_reporting.store import MissionStore
from target_reporting.transport import ImageReportServer, JsonReportServer


def main():
    parser = argparse.ArgumentParser(description="目标坐标与证据图接收端")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--image-port", type=int, default=5001)
    parser.add_argument("--output", default="./received_target_reports")
    parser.add_argument("--mission-id", default="competition_current")
    # roslaunch appends ROS remapping arguments such as __name:=... .
    args, _ros_remappings = parser.parse_known_args()

    store = MissionStore(args.output, "receiver", mission_id=args.mission_id)

    def save_event(event):
        added = store.append_event(event)
        if not added and not store.has_event(event["drone_id"], event["seq"]):
            return False
        pos = event.get("position", {})
        print()
        print(
            "[{timestamp:.3f}] {target_id} {target_type} result={result} "
            "channel=({x:.3f}, {y:.3f}, {z:.3f})".format(
                timestamp=float(event.get("timestamp", 0.0)),
                target_id=event.get("target_id", "-"),
                target_type=event.get("target_type", "-"),
                result=event.get("result", {}),
                x=float(pos.get("x", 0.0)), y=float(pos.get("y", 0.0)),
                z=float(pos.get("z", 0.0)),
            ), flush=True,
        )
        return True

    def save_image(image_id, data):
        path = store.save_image(image_id, data)
        print("[IMAGE] {} -> {}".format(image_id, path), flush=True)
        return True

    def show_observation(event):
        pos = event.get("position", {})
        print(
            "\r[LIVE] {target_id} {target_type} result={result} "
            "channel=({x:.3f}, {y:.3f}, {z:.3f}) hits={hits}    ".format(
                target_id=event.get("target_id", "-"),
                target_type=event.get("target_type", "-"),
                result=event.get("result", {}),
                x=float(pos.get("x", 0.0)), y=float(pos.get("y", 0.0)),
                z=float(pos.get("z", 0.0)), hits=int(event.get("hits", 0)),
            ), end="", flush=True,
        )

    json_server = JsonReportServer(args.host, args.port, save_event, observe=show_observation)
    image_server = ImageReportServer(args.host, args.image_port, save_image)
    json_server.start()
    image_server.start()
    stop = threading.Event()

    def request_stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)
    print("JSON监听 {}:{}，图片监听 {}:{}".format(args.host, json_server.port,
                                                   args.host, image_server.port), flush=True)
    try:
        while not stop.wait(0.5):
            pass
    finally:
        json_server.stop()
        image_server.stop()


if __name__ == "__main__":
    main()
