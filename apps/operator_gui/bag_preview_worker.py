"""Offline single-pass video review, adapted from export_rosbag_video.py.

Reads SQLite rosbag data in read-only mode. Never initializes a ROS node or
publishes recorded messages. Its private AVI cache lives only for this process.
"""
import argparse
import base64
import json
from pathlib import Path
import queue
import signal
import sqlite3
import sys
import tempfile
import threading
import time

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from teleop_core.contract import CAMERA_COLOR_TOPIC
import yaml

from camera_preview_worker import image_bgr

ORDER = (2, 0, 1)
FPS = 15


def emit(**data):
    print(json.dumps(data), flush=True)


def databases(bag):
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if meta.get('storage_identifier') != 'sqlite3':
        raise ValueError('暂不支持此 rosbag 存储格式')
    files = [(bag / name).resolve() for name in meta['relative_file_paths']]
    if not files or any(not p.is_relative_to(bag) or not p.is_file() for p in files):
        raise ValueError('rosbag 文件不完整')
    return files


def open_bag(path):
    return sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True)


def export(bag, cache, stop):
    """Same causal last-frame hold timeline as the reference converter."""
    names = [CAMERA_COLOR_TOPIC.format(index=i) for i in ORDER]
    files = databases(bag)
    first, last, found = [], [], set()
    for path in files:
        with open_bag(path) as db:
            for name, start, end in db.execute(
                'SELECT t.name, MIN(m.timestamp), MAX(m.timestamp) FROM messages m '
                'JOIN topics t ON m.topic_id=t.id WHERE t.name IN (?,?,?) GROUP BY t.name', names):
                found.add(name); first.append(start); last.append(end)
    if found != set(names): raise ValueError('上次样本缺少三路彩色图像')
    start, end = min(first), max(last)
    latest = [np.zeros((240, 320, 3), np.uint8) for _ in ORDER]
    paths = [cache / f'cam{i}.avi' for i in ORDER]
    writers = [cv2.VideoWriter(str(p), cv2.VideoWriter_fourcc(*'MJPG'), FPS, (320, 240)) for p in paths]
    count, next_frame, last_progress = [0]*3, 0, 0.
    try:
        if not all(w.isOpened() for w in writers): raise ValueError('本机视频编码器不可用')
        def until(seconds):
            nonlocal next_frame
            while next_frame / FPS < seconds:
                if stop.is_set(): raise InterruptedError()
                for writer, image in zip(writers, latest): writer.write(image)
                next_frame += 1
        for path in files:
            with open_bag(path) as db:
                query = ('SELECT m.timestamp,m.data,t.name FROM messages m JOIN topics t ON m.topic_id=t.id '
                         'WHERE t.name IN (?,?,?) ORDER BY m.timestamp,m.id')
                for received, data, name in db.execute(query, names):
                    if stop.is_set(): raise InterruptedError()
                    seconds = (received-start)/1e9
                    until(seconds)
                    image = image_bgr(deserialize_message(data, Image))
                    if image.ndim == 2 or image.shape[2] == 1:
                        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    canvas = np.zeros((240, 320, 3), np.uint8)
                    y, x = (240-image.shape[0])//2, (320-image.shape[1])//2
                    canvas[y:y+image.shape[0], x:x+image.shape[1]] = image
                    index = names.index(name); latest[index] = canvas; count[index] += 1
                    if time.monotonic()-last_progress > 1:
                        emit(event='decoding', elapsed=seconds, duration=(end-start)/1e9)
                        last_progress = time.monotonic()
        until((end-start)/1e9 + 1/FPS)
        if not all(count): raise ValueError('上次样本缺少有效图像')
    finally:
        for writer in writers: writer.release()
    emit(event='ready', cache=str(cache), source_counts=count, duration=(end-start)/1e9)
    return paths


def play(paths, stop, commands):
    """One synchronized timeline; commands never publish messages into ROS."""
    captures = [cv2.VideoCapture(str(path)) for path in paths]
    try:
        if not all(c.isOpened() for c in captures): raise ValueError('临时视频无法打开')
        total = min(int(c.get(cv2.CAP_PROP_FRAME_COUNT)) for c in captures)
        if total < 1: raise ValueError('临时视频没有图像')
        position, control_id, paused, ended = 0, 0, False, False
        deadline = time.monotonic()
        preview = False
        while not stop.is_set():
            try:
                command = commands.get(timeout=.02 if paused or ended else max(0., min(.02, deadline-time.monotonic())))
            except queue.Empty:
                command = None
            if command is not None:
                control_id = int(command.get('id', control_id))
                action = command.get('command')
                if action == 'pause': paused = True
                elif action in {'resume', 'play'}:
                    if ended or action == 'play': position = 0
                    paused, ended = False, False
                elif action == 'seek':
                    # Seek relative to the displayed frame, to the nearest cached frame.
                    current = position if preview else max(0, position-1)
                    position = max(0, min(total-1, round((current/FPS + float(command['seconds']))*FPS)))
                    ended, preview = False, True
                    paused = bool(command.get('paused', paused))
                for capture in captures: capture.set(cv2.CAP_PROP_POS_FRAMES, position)
                deadline = time.monotonic()
                emit(event='playback_state', control_id=control_id, paused=paused,
                     position=max(0, position-1)/FPS, ended=ended)
                if not commands.empty(): continue
            if (paused or ended) and not preview: continue
            if not preview and time.monotonic() < deadline: continue
            frames = [capture.read() for capture in captures]
            if not all(ok for ok, _ in frames):
                ended = True
                emit(event='playback_done', control_id=control_id, frames=position)
                continue
            for index, (_, frame) in zip(ORDER, frames):
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok: raise ValueError('回看图像编码失败')
                emit(camera=index, jpeg=base64.b64encode(jpeg).decode('ascii'),
                     control_id=control_id, position=position/FPS)
            position += 1
            preview = False
            deadline = time.monotonic()+1/FPS
    finally:
        for capture in captures: capture.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    args = parser.parse_args()
    stop, commands = threading.Event(), queue.Queue()
    def control():
        try:
            for line in sys.stdin:
                try:
                    command = json.loads(line)
                    if isinstance(command, dict): commands.put(command)
                except ValueError: continue
        finally: stop.set()
    threading.Thread(target=control, daemon=True).start()
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, lambda *_: stop.set())
    cv2.setNumThreads(1)
    # EOF on the parent's pipe also cleans up after terminal/GUI termination.
    with tempfile.TemporaryDirectory(prefix='gello-bag-preview-') as temporary:
        emit(event='cache_created', cache=temporary)
        try:
            paths = export(args.bag.resolve(), Path(temporary), stop)
            play(paths, stop, commands)
        except InterruptedError: pass
        except Exception as error:
            emit(event='error', message=str(error))


if __name__ == '__main__': main()
