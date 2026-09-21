from teleop_data_collector.camera_stream_health import StreamWindow


def test_stream_window_requires_full_stable_period_and_frequency():
    window = StreamWindow("/camera", 10.0)
    for index in range(51):
        window.observe(index * 0.1, max_gap_sec=0.15)

    assert window.ready(5.0, stable_sec=5.0, max_gap_sec=0.15)
    assert window.frequency_hz == 10.0


def test_stream_window_resets_after_large_gap():
    window = StreamWindow("/camera", 10.0)
    for index in range(21):
        window.observe(index * 0.1, max_gap_sec=0.15)
    window.observe(2.3, max_gap_sec=0.15)

    assert window.reset_count == 1
    assert window.sample_count == 1
    assert not window.ready(2.3, stable_sec=2.0, max_gap_sec=0.15)


def test_stream_window_becomes_unready_when_frames_stop():
    window = StreamWindow("/camera", 10.0)
    for index in range(21):
        window.observe(index * 0.1, max_gap_sec=0.15)

    assert window.ready(2.0, stable_sec=2.0, max_gap_sec=0.15)
    assert not window.ready(2.2, stable_sec=2.0, max_gap_sec=0.15)
    assert "STALE" in window.describe(2.2, max_gap_sec=0.15)
