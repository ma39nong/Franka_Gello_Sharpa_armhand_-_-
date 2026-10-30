"""Run in the ROS environment, without importing the Qt application package."""
from pathlib import Path
from types import SimpleNamespace
import runpy
import pytest


def test_thumbnail_preserves_source_and_handles_padded_rgb():
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    pytest.importorskip('rclpy')
    root = Path(__file__).resolve().parents[2]
    thumbnail = runpy.run_path(str(root / 'apps/operator_gui/camera_preview_worker.py'))['thumbnail']
    raw = np.zeros((480, 640 * 3 + 8), dtype=np.uint8)
    raw[:, 1:640*3:3] = 220
    data = bytearray(raw.tobytes())
    message = SimpleNamespace(width=640, height=480, step=1928, encoding='rgb8', data=data)
    before = bytes(data)
    jpeg = thumbnail(message)
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert image.shape == (240, 320, 3)
    assert image[:, :, 1].mean() > 200
    assert bytes(data) == before
    message.step = 1
    with pytest.raises(ValueError, match='stride'): thumbnail(message)
    message.encoding = '16UC1'
    with pytest.raises(ValueError, match='format'): thumbnail(message)
