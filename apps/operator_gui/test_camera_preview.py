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
