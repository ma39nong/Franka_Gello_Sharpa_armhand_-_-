"""Qt modes and lifetime for read-only live preview and last-bag review."""
import base64
import json
import runpy
import shutil
import sqlite3
import tempfile
import time
from pathlib import Path

from PySide6.QtCore import QProcess, QProcessEnvironment, Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSizePolicy, QPushButton

COLOR_TOPIC = runpy.run_path(str(Path(__file__).resolve().parents[2] / 'ros_ws/src/teleop_core/teleop_core/contract.py'))['CAMERA_COLOR_TOPIC']


class CameraPreview(QWidget):
    notice = Signal(str)
    HEIGHT = 260
    CAMERAS = ((2, '左手腕相机 · cam2'), (0, '中间相机 · cam0'), (1, '右手腕相机 · cam1'))

    def __init__(self, root, parent=None):
        super().__init__(parent)
        self.root = Path(root)
        self.process = None
        self.buffer = b''
        self.frames = {}
        self.labels = {}
        self.expanded = True
        self.mode = None
        self.playing = False
        self.last_bag = None
        self.review_reason = '没有可回看的已完成样本'
        self.cache_path = None
        self._topic_check = None
        self._last_file_check = 0.
        self.setFixedHeight(self.HEIGHT)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        images = QHBoxLayout()
        for index, title in self.CAMERAS:
            image = QLabel('等待图像')
            image.setToolTip(title)
            image.setAlignment(Qt.AlignCenter)
            image.setMinimumSize(0, 0)
            image.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            image.setStyleSheet('background: #171923; color: #c7c4d2; border-radius: 4px;')
            images.addWidget(image, 1)
            self.labels[index] = image
        layout.addLayout(images, 1)
        controls = QHBoxLayout()
        controls.addStretch()
        self.live_button = QPushButton('当前视频流')
        self.live_button.setCheckable(True)
        self.last_button = QPushButton('查看上次')
        for button in (self.live_button, self.last_button):
            button.setFocusPolicy(Qt.NoFocus)
            button.setStyleSheet('padding: 2px 8px;')
            controls.addWidget(button)
        layout.addLayout(controls)
        self.live_button.clicked.connect(self.select_live)
        self.last_button.clicked.connect(self.select_last)
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self.update_controls()

    def update_controls(self):
        available = (self.root / 'portable_deps/litchi_hardware/.pixi/envs/ros2/bin/python').is_file()
        self.live_button.setEnabled(self.expanded and available)
        self.last_button.setEnabled(self.expanded and available and self.last_bag is not None and not self.playing)
        self.live_button.setToolTip('选中并展开时读取实时图像；取消选中或折叠时停止读取')
        if not self.playing: self.last_button.setToolTip(self.review_reason)

    def update_collection_status(self, status):
        """Use the collector's authoritative last episode, never scan for an older success."""
        candidate = None
        reason = '没有可回看的已完成样本'
        busy = status.get('active') or str(status.get('state', '')).upper() in {'RECORDING','STARTING','STOPPING','VALIDATING'}
        storage = status.get('storage') or {}
        if busy: reason = '正在录制或校验，结束后才可查看上次'
        elif (status.get('quality') == '报错' or status.get('failures')
              or str(status.get('state', '')).upper() in {'ERROR','FAULT','FAILED','INCOMPLETE','INTERRUPTED'}):
            reason = '上次样本为报错或未完成，不可回看'
        elif storage.get('state') in {'archiving','failed'}: reason = '样本正在归档或保存失败'
        elif status.get('finalized') and status.get('local_bag_available', True):
            raw = status.get('last_bag') or storage.get('path')
            if raw:
                path = Path(raw).expanduser()
                if not path.is_dir() and path.is_relative_to('/collection_data'):
                    values = self.deployment()
                    path = Path(values.get('TELEOP_DATA_ROOT', str(Path.home()/'franka_teleop_data'))) / path.relative_to('/collection_data')
                bag = path if (path/'metadata.yaml').is_file() else path/'rosbag'
                try:
                    files = sorted(bag.glob('*.db3'))
                    key = (str(bag), (bag/'metadata.yaml').stat().st_mtime_ns, tuple(str(p) for p in files))
                    if self._topic_check is None or self._topic_check[0] != key:
                        names = set()
                        for file in files:
                            with sqlite3.connect(file.resolve().as_uri()+'?mode=ro&immutable=1', uri=True, timeout=.1) as db:
                                names.update(row[0] for row in db.execute('SELECT name FROM topics'))
                        needed = {COLOR_TOPIC.format(index=i) for i in (2, 0, 1)}
                        self._topic_check = (key, needed <= names)
                    if self._topic_check[1]: candidate, reason = bag.resolve(), '单次播放上一段已完成样本，不向 ROS 回放消息'
                    else: reason = '上次样本没有完整的三路彩色图像'
                except (OSError, sqlite3.Error, ValueError): reason = '上次样本不在本机或 rosbag 文件不完整'
        if self.mode == 'last' and candidate != self.last_bag:
            self.stop(); self.mode = None
        self.last_bag, self.review_reason = candidate, reason
        self.update_controls()

    def deployment(self):
        values = {}
        path = self.root / 'docker/.env'
        if path.exists():
            for line in path.read_text().splitlines():
                key, sep, value = line.partition('=')
                if sep: values[key.strip()] = value.strip().strip('"\'')
        return values

    def select_live(self, checked):
        if not self.expanded: return
        self.stop()
        self.mode = 'live' if checked else None
        self.timer.setInterval(200)
        if checked: self.start()
        self.update_controls()

    def select_last(self):
        if not self.last_button.isEnabled(): return
        self.live_button.setChecked(False)
        if self.mode == 'last' and self.process is not None and self.process.state() == QProcess.Running:
            self.playing = True
            self.process.write(b'play\n')
        else:
            self.stop()
            self.mode, self.playing = 'last', True
            for label in self.labels.values(): label.setText('正在准备回看…')
            self.start('bag_preview_worker.py', [str(self.last_bag)])
        self.timer.setInterval(33)
        self.update_controls()

    def start(self, worker='camera_preview_worker.py', arguments=()):
        python = self.root / 'portable_deps/litchi_hardware/.pixi/envs/ros2/bin/python'
        if not python.exists():
            for label in self.labels.values(): label.setText('ROS 预览环境不可用')
            return
        env = QProcessEnvironment.systemEnvironment()
        deployment = self.deployment()
        env.insert('PYTHONPATH', str(self.root / 'ros_ws/src/teleop_core'))
        env.insert('AMENT_PREFIX_PATH', str(python.parent.parent))
        env.insert('ROS_DOMAIN_ID', deployment.get('TELEOP_ROS_DOMAIN_ID', '0'))
        env.insert('ROS_LOCALHOST_ONLY', '1')
        env.insert('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
        env.insert('CYCLONEDDS_URI', (self.root / 'config/cyclonedds.xml').as_uri())
        env.insert('OPENBLAS_NUM_THREADS', '1')
        process = QProcess(self)
        process.setProcessEnvironment(env)
        process.readyReadStandardOutput.connect(lambda: self.read_frames(process))
        process.readyReadStandardError.connect(lambda: process.readAllStandardError())
        process.errorOccurred.connect(lambda error: self.unavailable(process))
        process.finished.connect(lambda *_: self.unavailable(process))
        self.process = process
        self.buffer = b''
        args = [str(self.root / 'apps/operator_gui' / worker), *arguments]
        cpuset = deployment.get('HOUSEKEEPING_CPUSET')
        if cpuset:
            process.start('/usr/bin/taskset', ['-c', cpuset, str(python), *args])
        else:
            process.start(str(python), args)

    def unavailable(self, process=None):
        if process is not None and process is not self.process: return
        self.playing = False
        self.frames.clear()
        for label in self.labels.values(): label.setText('回看未完成' if self.mode == 'last' else '视频预览未连接')
        self.update_controls()

    def read_frames(self, process=None):
        if self.process is None or (process is not None and process is not self.process): return
        self.buffer += bytes(self.process.readAllStandardOutput())
        if len(self.buffer) > 2_000_000:
            self.buffer = b''
            return
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                row = json.loads(line)
                if 'event' in row:
                    event = row['event']
                    if event == 'cache_created': self.cache_path = Path(row['cache'])
                    elif event == 'decoding': self.last_button.setToolTip(f"正在离线解码：{row['elapsed']:.1f} / {row['duration']:.1f} 秒；可切回当前视频流取消")
                    elif event == 'playback_done':
                        self.playing = False
                        self.notice.emit('上次样本播放结束；可再次点击查看上次单次播放')
                        self.update_controls()
                    elif event == 'error':
                        self.playing = False
                        self.notice.emit('查看上次失败：'+row.get('message','未知错误'))
                        self.update_controls()
                    continue
                index = row['camera']
                if index not in self.labels: continue
                image = QImage.fromData(base64.b64decode(row['jpeg'], validate=True))
                if not image.isNull(): self.frames[index] = (time.monotonic(), QPixmap.fromImage(image))
            except (ValueError, KeyError, TypeError): continue

    def refresh(self):
        if not self.expanded or not self.isVisible(): return
        if time.monotonic()-self._last_file_check > 2:
            self._last_file_check = time.monotonic()
            if self.last_bag is not None and not (self.last_bag/'metadata.yaml').is_file():
                if self.mode == 'last': self.stop(); self.mode = None
                self.last_bag = None
                self.review_reason = '上次样本已不在本机'
                self.update_controls()
        if self.mode == 'live' and self.process is None: self.start()
        for index, label in self.labels.items():
            frame = self.frames.get(index)
            if frame is None: continue
            when, pixmap = frame
            if self.mode == 'live' and time.monotonic() - when > 2:
                label.clear(); label.setText('等待新图像')
            else:
                label.setPixmap(pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def stop(self):
        self.frames.clear()
        self.buffer = b''
        if self.process is not None:
            process, self.process = self.process, None
            if self.mode == 'last':
                process.closeWriteChannel()
                if not process.waitForFinished(100):
                    process.terminate()
                    if not process.waitForFinished(100): process.kill(); process.waitForFinished(100)
            else:
                process.kill(); process.waitForFinished(1000)
            process.deleteLater()
        if self.cache_path is not None:
            # Only delete the worker's private cache, never a recording path.
            path = self.cache_path.resolve()
            if path.parent == Path(tempfile.gettempdir()).resolve() and path.name.startswith('gello-bag-preview-'):
                shutil.rmtree(path, ignore_errors=True)
            self.cache_path = None
        self.playing = False
        for label in self.labels.values(): label.clear(); label.setText('等待图像')

    def set_expanded(self, expanded):
        self.expanded = expanded
        self.setVisible(expanded)
        if expanded: self.timer.start()
        else:
            self.timer.stop()
            self.stop()
            if self.mode == 'last': self.mode = None
        self.update_controls()

    def close_preview(self):
        self.timer.stop()
        self.stop()
