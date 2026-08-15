#!/usr/bin/env python3
"""Persist one LDOP flight's target and odometry data for later analysis."""

import argparse
import csv
import datetime as dt
import os
import signal
import threading
import time

import rospy
from ldop.msg import DynamicObjectArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def wall_iso(timestamp):
    return dt.datetime.fromtimestamp(timestamp).astimezone().isoformat(timespec="milliseconds")


def ros_seconds(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


class FlightLogger:
    def __init__(self, log_dir):
        self.log_dir = os.path.abspath(log_dir)
        os.makedirs(self.log_dir, exist_ok=True)
        self.lock = threading.Lock()
        self.last_ids = None
        self.dynamic_frames = None
        self.dynamic_objects = None
        self.id_events = None
        self.odometry = None
        self.marker_counts = None
        self.files = []
        self.raw_boxes = 0
        self.stable_boxes = 0

        self._open_csv(
            "dynamic_frames.csv",
            ["wall_time", "wall_iso", "ros_time", "ids", "object_count"],
            "dynamic_frames",
        )
        self._open_csv(
            "dynamic_objects.csv",
            [
                "wall_time",
                "wall_iso",
                "ros_time",
                "id",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "size_x",
                "size_y",
                "size_z",
                "object_class",
                "motion_model",
            ],
            "dynamic_objects",
        )
        self._open_csv(
            "id_events.csv",
            ["wall_time", "wall_iso", "ros_time", "previous_ids", "current_ids"],
            "id_events",
        )
        self._open_csv(
            "odometry.csv",
            ["wall_time", "wall_iso", "ros_time", "x", "y", "z", "vx", "vy", "vz"],
            "odometry",
        )
        self._open_csv(
            "marker_counts.csv",
            ["wall_time", "wall_iso", "raw_candidate_boxes", "stable_track_boxes"],
            "marker_counts",
        )

        with open(os.path.join(self.log_dir, "metadata.txt"), "w", encoding="utf-8") as metadata:
            metadata.write("LDOP flight log\n")
            metadata.write("started_wall_time=%s\n" % wall_iso(time.time()))
            metadata.write("pid=%d\n" % os.getpid())
            metadata.write("dynamic_topic=/ldop/dynamic_objects\n")
            metadata.write("odom_topic=/Odometry\n")
            metadata.write("raw_marker_topic=/ldop/dynamic_object_markers\n")
            metadata.write("stable_marker_topic=/ldop/dynamic_track_markers\n")

    def _open_csv(self, filename, header, attribute):
        handle = open(
            os.path.join(self.log_dir, filename),
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        writer = csv.writer(handle)
        writer.writerow(header)
        setattr(self, attribute, writer)
        self.files.append(handle)

    def dynamic_callback(self, message):
        wall_time = time.time()
        ros_time = ros_seconds(message.header.stamp)
        objects = list(message.objects)
        ids = tuple(sorted(int(obj.id) for obj in objects))
        ids_text = ";".join(str(value) for value in ids)

        with self.lock:
            self.dynamic_frames.writerow(
                [wall_time, wall_iso(wall_time), ros_time, ids_text, len(objects)]
            )
            if self.last_ids is not None and ids != self.last_ids:
                self.id_events.writerow(
                    [
                        wall_time,
                        wall_iso(wall_time),
                        ros_time,
                        ";".join(str(value) for value in self.last_ids),
                        ids_text,
                    ]
                )
                print(
                    "ID集合变化: %s -> %s"
                    % (list(self.last_ids), list(ids)),
                    flush=True,
                )

            for obj in objects:
                state = list(obj.model_state)
                position = state[0:3] if len(state) >= 3 else [float("nan")] * 3
                velocity = state[3:6] if len(state) >= 6 else [float("nan")] * 3
                self.dynamic_objects.writerow(
                    [
                        wall_time,
                        wall_iso(wall_time),
                        ros_time,
                        int(obj.id),
                        position[0],
                        position[1],
                        position[2],
                        velocity[0],
                        velocity[1],
                        velocity[2],
                        obj.size.x,
                        obj.size.y,
                        obj.size.z,
                        int(obj.object_class),
                        int(obj.motion_model_type),
                    ]
                )
            self.last_ids = ids

    def odometry_callback(self, message):
        wall_time = time.time()
        with self.lock:
            position = message.pose.pose.position
            velocity = message.twist.twist.linear
            self.odometry.writerow(
                [
                    wall_time,
                    wall_iso(wall_time),
                    ros_seconds(message.header.stamp),
                    position.x,
                    position.y,
                    position.z,
                    velocity.x,
                    velocity.y,
                    velocity.z,
                ]
            )

    def marker_callback(self, message, stable):
        count = sum(
            1
            for marker in message.markers
            if marker.action not in (Marker.DELETE, Marker.DELETEALL)
            and marker.ns
            == ("dynamic_track_bbox" if stable else "dynamic_object_bbox")
        )
        wall_time = time.time()
        with self.lock:
            if stable:
                self.stable_boxes = count
            else:
                self.raw_boxes = count
            self.marker_counts.writerow(
                [wall_time, wall_iso(wall_time), self.raw_boxes, self.stable_boxes]
            )

    def close(self):
        with self.lock:
            for handle in self.files:
                handle.flush()
                handle.close()
            self.files = []
        with open(os.path.join(self.log_dir, "finished.txt"), "w", encoding="utf-8") as finished:
            finished.write("finished_wall_time=%s\n" % wall_iso(time.time()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", required=True)
    args = parser.parse_args()

    logger = FlightLogger(args.log_dir)

    def handle_signal(signum, _frame):
        rospy.signal_shutdown("signal %d" % signum)

    rospy.init_node("ldop_flight_logger", anonymous=True, disable_signals=True)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    rospy.Subscriber("/ldop/dynamic_objects", DynamicObjectArray, logger.dynamic_callback, queue_size=100)
    rospy.Subscriber("/Odometry", Odometry, logger.odometry_callback, queue_size=100)
    rospy.Subscriber(
        "/ldop/dynamic_object_markers",
        MarkerArray,
        lambda message: logger.marker_callback(message, False),
        queue_size=20,
    )
    rospy.Subscriber(
        "/ldop/dynamic_track_markers",
        MarkerArray,
        lambda message: logger.marker_callback(message, True),
        queue_size=20,
    )
    rospy.on_shutdown(logger.close)
    print("LDOP 飞行日志记录已启动: %s" % logger.log_dir, flush=True)
    rospy.spin()


if __name__ == "__main__":
    main()
