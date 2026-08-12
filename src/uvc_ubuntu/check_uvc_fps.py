#!/usr/bin/env python3
import subprocess
import time
import os
import signal

WIDTH = 384
HEIGHT = 288
FRAME_SIZE = WIDTH * HEIGHT * 2

p = subprocess.Popen(
    ["./uvc_demo"],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    preexec_fn=os.setsid
)

frames = 0
short_reads = 0
start = time.time()
duration = 10.0

try:
    while time.time() - start < duration:
        data = p.stdout.read(FRAME_SIZE)
        if len(data) == FRAME_SIZE:
            frames += 1
        else:
            short_reads += 1
            break
finally:
    os.killpg(os.getpgid(p.pid), signal.SIGTERM)

elapsed = time.time() - start
print("frames:", frames)
print("elapsed:", elapsed)
print("fps:", frames / elapsed)
print("short_reads:", short_reads)
