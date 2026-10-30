"""Preview-specific isolation and image conversion checks; no camera required."""
import sys
from pathlib import Path

import pytest


def test_live_requires_explicit_selection_and_eligible_review_is_gated(tmp_path):
    import sqlite3
    from unittest.mock import Mock
    from PySide6.QtWidgets import QApplication
    from apps.operator_gui.camera_preview import CameraPreview, COLOR_TOPIC
    app = QApplication.instance() or QApplication([])
    root = Path(__file__).resolve().parents[2]
    panel = CameraPreview(root)
    panel.start = Mock()
    bag = tmp_path/'episode1'; bag.mkdir()
    (bag/'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    with sqlite3.connect(bag/'episode1_0.db3') as db:
        db.execute('CREATE TABLE topics (name TEXT)')
        db.executemany('INSERT INTO topics VALUES (?)', [(COLOR_TOPIC.format(index=i),) for i in (2,0,1)])
    valid = dict(state='READY', finalized=True, local_bag_available=True, last_bag=str(bag), quality='一般')
    try:
        panel.show(); panel.refresh()
        panel.start.assert_not_called()
        assert not panel.live_button.isChecked() and not panel.last_button.isEnabled()
        panel.update_collection_status(valid)
        assert panel.last_button.isEnabled()
        for changed in ({'active':True}, {'state':'VALIDATING'}, {'state':'FAILED'}, {'quality':'报错'},
                        {'failures':['gap']}, {'finalized':False}, {'local_bag_available':False},
                        {'last_bag':str(tmp_path/'missing')}):
            panel.update_collection_status(dict(valid,**changed))
            assert not panel.last_button.isEnabled(), changed
        panel.update_collection_status(valid)
        panel.live_button.click()
        panel.start.assert_called_once()
        assert panel.mode=='live'
        panel.live_button.click()
        assert panel.mode is None
        panel.set_expanded(False)
        assert not panel.live_button.isEnabled() and not panel.last_button.isEnabled()
    finally:
        panel.close_preview();panel.close();app.processEvents()


def test_collapse_terminates_owned_reader_and_clears_frames():
    pytest.importorskip('PySide6')
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import QProcess
    from apps.operator_gui.camera_preview import CameraPreview
    app = QApplication.instance() or QApplication([])
    panel = CameraPreview(Path('/missing/preview/environment'))
    reader = QProcess(panel)
    reader.start(sys.executable, ['-c', 'import time; time.sleep(60)'])
    assert reader.waitForStarted(2000)
    panel.process = reader
    panel.frames[2] = object()
    try:
        panel.set_expanded(False)
        assert panel.process is None and not panel.frames
        assert reader.state() == QProcess.NotRunning
        assert not panel.timer.isActive()
        panel.set_expanded(True)
        assert panel.timer.isActive() and panel.process is None
    finally:
        panel.close_preview()
        panel.close()
        app.processEvents()


def test_episode_picker_preserves_mode_on_cancel_or_invalid_sample(tmp_path, monkeypatch):
    import json
    import sqlite3
    from unittest.mock import Mock
    from PySide6.QtWidgets import QApplication, QFileDialog
    from apps.operator_gui.camera_preview import CameraPreview, COLOR_TOPIC
    app = QApplication.instance() or QApplication([])
    panel = CameraPreview(Path(__file__).resolve().parents[2])
    episode = tmp_path/'episode'; bag = episode/'rosbag'; bag.mkdir(parents=True)
    (bag/'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    state = episode/'collection_state.json'
    with sqlite3.connect(bag/'test.db3') as db:
        db.execute('CREATE TABLE topics (name TEXT)')
        db.executemany('INSERT INTO topics VALUES (?)', [(COLOR_TOPIC.format(index=i),) for i in (2,0,1)])
    panel.begin_review = Mock()
    try:
        monkeypatch.setattr(QFileDialog, 'getExistingDirectory', lambda *args: '')
        panel.choose_button.click()
        panel.begin_review.assert_not_called()
        monkeypatch.setattr(QFileDialog, 'getExistingDirectory', lambda *args: str(episode))
        for status in ({'finalized':False}, {'finalized':True,'failures':['gap']},
                       {'finalized':True,'quality':'报错'}):
            state.write_text(json.dumps(status))
            panel.choose_button.click()
            panel.begin_review.assert_not_called()
        state.write_text(json.dumps({'finalized':True,'state':'finalized'}))
        panel.choose_button.click()
        panel.begin_review.assert_called_once_with(bag.resolve(), 'selected')
        panel.update_collection_status({'active':True})
        assert not panel.choose_button.isEnabled()
        panel.set_expanded(False)
        assert not panel.pause_button.isEnabled() and not panel.back_button.isEnabled()
    finally:
        panel.close_preview(); panel.close(); app.processEvents()


def test_pause_discards_queued_frames_and_seek_uses_new_control_id():
    import base64
    import json
    from unittest.mock import Mock
    from PySide6.QtCore import QByteArray, QBuffer, QIODevice
    from PySide6.QtGui import QImage
    from PySide6.QtWidgets import QApplication
    from apps.operator_gui.camera_preview import CameraPreview
    app = QApplication.instance() or QApplication([])
    panel = CameraPreview(Path(__file__).resolve().parents[2])
    image = QImage(2,2,QImage.Format_RGB32);image.fill(0)
    data = QByteArray(); buffer = QBuffer(data);buffer.open(QIODevice.WriteOnly)
    assert image.save(buffer, 'PNG')
    jpeg = base64.b64encode(bytes(data)).decode()
    reader = Mock()
    panel.process = reader
    panel.mode, panel.media_ready, panel.playing = 'selected', True, True
    panel.update_controls()
    try:
        panel.pause_button.click()
        assert panel.paused
        command = json.loads(bytes(reader.write.call_args.args[0]))
        assert command['command']=='pause'
        reader.readAllStandardOutput.return_value = (json.dumps(dict(camera=2,jpeg=jpeg,position=1.,control_id=0))+'\n').encode()
        panel.read_frames(reader)
        assert not panel.frames
        panel.forward_button.click()
        command = json.loads(bytes(reader.write.call_args.args[0]))
        assert command==dict(command='seek', id=2, seconds=.5, paused=True)
        reader.readAllStandardOutput.return_value = (json.dumps(dict(camera=2,jpeg=jpeg,position=1.5,control_id=2))+'\n').encode()
        panel.read_frames(reader)
        assert panel.positions[2]==1.5 and 2 in panel.frames and panel.paused
    finally:
        panel.process = None
        panel.close_preview();panel.close();app.processEvents()
