"""Read-only ROS image subscriber; emits bounded JPEG previews over stdout.

Runs in the host ROS Python environment, independently of Qt and the recorder.
No camera device access, ROS publishers, services, or recording controls.
"""
import base64
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from teleop_core.contract import CAMERA_COLOR_TOPIC


def image_bgr(message):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    count = channels.get(message.encoding)
    if count is None or not 0 < message.width <= 8192 or not 0 < message.height <= 8192:
        raise ValueError('Unsupported preview image format')
    row_bytes = message.width * count
    if message.step < row_bytes or len(message.data) != message.step * message.height:
        raise ValueError('Invalid image stride/data length')
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :row_bytes].reshape(message.height, message.width, count)
    scale = min(320 / message.width, 240 / message.height, 1.)
    pixels = cv2.resize(pixels, (max(1, round(message.width * scale)), max(1, round(message.height * scale))))
    if message.encoding == 'rgb8': pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    elif message.encoding == 'rgba8': pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    elif message.encoding == 'bgra8': pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    return pixels


def thumbnail(message):
    pixels = image_bgr(message)
    ok, jpeg = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if not ok: raise ValueError('JPEG encoding failed')
    return jpeg.tobytes()


class PreviewSubscriber(Node):
    def __init__(self):
        super().__init__('operator_camera_preview')
        self.latest = {}
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [self.create_subscription(
            Image,
            CAMERA_COLOR_TOPIC.format(index=index),
            lambda msg, index=index: self.latest.update({index: (time.monotonic(), msg)}), qos)
            for index in (2, 0, 1)]
        self.create_timer(.2, self.flush_latest)

    def flush_latest(self):
        frames, self.latest = self.latest, {}
        for index, (received, message) in frames.items():
            if time.monotonic() - received > .5: continue
            try:
                jpeg = thumbnail(message)
                print(json.dumps({'camera': index, 'jpeg': base64.b64encode(jpeg).decode('ascii')}), flush=True)
            except (ValueError, cv2.error) as error:
                self.get_logger().warning(str(error))


def main():
    cv2.setNumThreads(1)
    rclpy.init()
    node = PreviewSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
