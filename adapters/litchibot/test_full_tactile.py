from types import SimpleNamespace
from unittest.mock import Mock
import threading
import pytest

from adapters.litchibot.full_tactile import FullTactilePublisher,fast_tactile_image


@pytest.mark.parametrize('size',[1024,307200])
def test_fast_image_preserves_fields_and_serialized_payload(size):
    pytest.importorskip('rclpy')
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message,deserialize_message
    from sharpa_driver.messages import tactile_image_message
    image=SimpleNamespace(height=1,width=size,encoding='mono8',step=size,
        data=bytes(range(256))*(size//256))
    stamp=Time(sec=123,nanosec=456)
    original=tactile_image_message(image,frame_id='left/index',stamp=stamp)
    fast=fast_tactile_image(image,frame_id='left/index',stamp=stamp)
    # CDR alignment padding is unspecified and may contain different bytes.
    # Compare every field after serialization, including the complete payload.
    assert original==fast
    assert deserialize_message(serialize_message(original),type(original))==original
    assert deserialize_message(serialize_message(fast),type(fast))==original


def test_slow_publication_does_not_block_poll_or_move_sdk_to_worker(monkeypatch):
    pytest.importorskip('rclpy')
    import sharpa_driver.messages as messages
    from litchi_hardware.core.types import HandSide
    from litchi_hardware.hardware.sharpa.tactile import SharpaFinger
    monkeypatch.setattr(messages,'validate_tactile_route',lambda *a,**k:'left/index')
    owner=threading.get_ident();reads=[];frames=iter([1,2,3])
    def read(*a,**k):reads.append(threading.get_ident());return next(frames)
    hand=SimpleNamespace(read_tactile=read)
    node=SimpleNamespace(_hands={HandSide.LEFT:hand},
        _tactile_publishers={(HandSide.LEFT,SharpaFinger.INDEX):{}},
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:None)))
    publisher=FullTactilePublisher(node)
    started=threading.Event();release=threading.Event();delivered=threading.Event();seen=[]
    def slow(frame,*a):
        if frame==1:started.set();assert release.wait(2)
        seen.append(frame)
        if frame==3:delivered.set()
    publisher.publish=slow
    try:
        publisher.poll();assert started.wait(1)
        publisher.poll();publisher.poll()  # Must return while the worker is blocked.
        assert reads==[owner,owner,owner]
        release.set();assert delivered.wait(1)
        assert seen==[1,3]  # Bounded latest snapshot, no stale-image FIFO.
    finally:release.set();publisher.close()


@pytest.mark.parametrize('count',[1,9,10])
def test_batches_preserve_every_channel_once_per_round_on_sdk_owner(monkeypatch,count):
    pytest.importorskip('rclpy')
    import sharpa_driver.messages as messages
    monkeypatch.setattr(messages,'validate_tactile_route',lambda *a,**k:'unused')
    # No frames needed: prove exactly which nonblocking SDK reads are made.
    from enum import Enum
    class Side(Enum):LEFT='left'
    owner=threading.get_ident();reads=[]
    def read(finger,**kwargs):
        assert threading.get_ident()==owner and kwargs=={'timeout_s':0.0}
        reads.append(finger);return None
    node=SimpleNamespace(_hands={Side.LEFT:SimpleNamespace(read_tactile=read)},
        _tactile_publishers={(Side.LEFT,i):{} for i in range(count)})
    publisher=FullTactilePublisher(node,batch_size=2)
    try:
        for _ in range(2):
            round_start=len(reads)
            for _ in range((count+1)//2):
                before=len(reads);publisher.poll();assert len(reads)-before<=2
            assert reads[round_start:]==list(range(count))
    finally:publisher.close()
