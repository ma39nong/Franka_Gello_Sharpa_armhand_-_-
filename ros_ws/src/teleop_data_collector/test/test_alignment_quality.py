"""Synthetic metadata only. Optional differential tests call the real converter."""
import os
from pathlib import Path
from types import SimpleNamespace
import pytest
from teleop_core.contract import COMMAND_STATUS_TOPIC as STATUS, VALIDATED_COMMAND_TOPIC as ACTION, LEFT_COMMAND_JOINT_NAMES, RIGHT_COMMAND_JOINT_NAMES
from teleop_data_collector.alignment_quality import AlignmentQuality, AlignmentPolicy, NS

TOPICS = [ACTION, STATUS, '/left/franka/joint_states', '/right/franka/joint_states']
TOPICS += [f'/sharpa/{s}/{k}' for s in ('left','right') for k in ('command','joint_states')]
TOPICS += [f'/cam{i}/color/image_raw' for i in range(3)] + ['/cam0/depth/image_raw']
TOPICS += [f'/sharpa/{s}/tactile/{f}/{k}' for s in ('left','right') for f in ('thumb','index','middle','ring','pinky') for k in ('wrench','deformation')]


def message(topic, active=('left','right')):
    return SimpleNamespace(name=LEFT_COMMAND_JOINT_NAMES + RIGHT_COMMAND_JOINT_NAMES,
                           accepted_sides=list(active), faults=[])


def fixture(duration=4, offset=0):
    result={}
    for topic in TOPICS:
        rate=60 if '/tactile/' in topic else 20 if topic.startswith('/cam0/') else 30
        result[topic]=[(NS+offset+i*NS//rate,message(topic)) for i in range(rate*duration+1)]
    return result


def gap(raw,topic,ms,phase=0):
    left=3*NS+round(phase*1_000_000);right=left+round(ms*1_000_000)
    old=raw[topic]
    raw[topic]=[(t,m) for t,m in old if t<left]+[(left,message(topic)),(right,message(topic))]+[(t,m) for t,m in old if t>right]


def run(raw, *, delays=None, policy=None, boundaries=()):
    policy=policy or AlignmentPolicy()
    quality=AlignmentQuality(list(raw),policy,compact_boundaries_ns=boundaries)
    events=sorted((t/NS+(delays(topic,t) if delays else 0),topic,t,m) for topic,series in raw.items() for t,m in series)
    for receive,topic,stamp,msg in events:
        quality.observe(topic,msg,stamp,receive)
        quality.advance(receive)
    start=max(series[0][0] for series in raw.values())+int(policy.trim_start_sec*NS)
    end=min(series[-1][0] for topic,series in raw.items() if topic!=ACTION)-int(policy.trim_end_sec*NS)
    quality.advance(events[-1][0]+1,final=True,start_ns=start,end_ns=end)
    return quality, start, end


def oracle(raw,start,end,*,boundaries=()):
    root=os.environ.get('CONVERSION_ORACLE_ROOT')
    if not root:
        pytest.skip('Set CONVERSION_ORACLE_ROOT to the pinned, read-only converter')
    import sys
    sys.path.insert(0,str(Path(root)/'src'))
    import numpy as np
    from convest.config import load_config
    from convest.sources.gello_rosbag2 import Series, timestamp_policy
    from convest.sources.robot_rosbag2 import modalities, STATUS_MESSAGE_TYPES
    from convest.recipes.multimodal import prepare, output_segments
    from convest.quality import RecordingRejected
    streams={}
    mapping={'/left/franka/joint_states':'state.left_arm','/right/franka/joint_states':'state.right_arm'}
    for side in ('left','right'):
        mapping[f'/sharpa/{side}/joint_states']=f'state.{side}_hand'
        mapping[f'/sharpa/{side}/command']=f'action.{side}_hand'
    mapping.update({f'/cam{i}/color/image_raw':f'cam{i}' for i in range(3)})
    mapping.update({v['topic']:key for key,v in modalities('sharpa').items()})
    for topic,key in mapping.items():
        dim=14 if key.startswith('state.') and key.endswith('arm') else 44 if key.startswith('state.') else 22 if key.startswith('action.') else 1
        streams[key]=Series.build([(t,np.zeros(dim)) for t,m in raw[topic]])
    for side,names in [('left',LEFT_COMMAND_JOINT_NAMES),('right',RIGHT_COMMAND_JOINT_NAMES)]:
        streams[f'action.{side}_arm']=Series.build([(t,np.zeros(7)) for t,m in raw[ACTION] if set(names).issubset(m.name)])
        streams[f'status.{side}_arm']=Series.build([(t,(side in m.accepted_sides,not bool(m.faults))) for t,m in raw[STATUS]])
    source=SimpleNamespace(read_streams=lambda bag,contract:streams,STATUS_MESSAGE_TYPES=STATUS_MESSAGE_TYPES,timestamp_policy=timestamp_policy)
    bag=SimpleNamespace(compact_time_map=SimpleNamespace(compact_boundaries=boundaries,unmodified_payload_types=()) if boundaries else None)
    cfg=load_config(Path(root)/'configs/sharpa.yaml')
    item={'source_start_ns':start,'source_end_ns':end,'path':'synthetic-test-only'}
    _,aligned=prepare(source,bag,item,cfg,{})
    try:
        output_segments(item,aligned,cfg)
        passed=True
    except RecordingRejected:
        passed=False
    return passed,aligned


def compare(raw,**kwargs):
    quality,start,end=run(raw,**kwargs)
    passed,aligned=oracle(raw,start,end,boundaries=kwargs.get("boundaries",()))
    assert (quality.first_failure is None)==passed, (quality.report(),aligned.report)
    assert quality.valid_count==int(aligned.valid_mask.sum())
    return quality,passed


def test_normal_bounded_and_causal():
    quality,_,_=run(fixture())
    assert quality.first_failure is None and quality.valid_count>60
    assert quality.peak_samples<100 and quality.accepted_rgb_frames==0


@pytest.mark.parametrize('topic',['/sharpa/left/joint_states','/sharpa/right/joint_states','/sharpa/left/command','/sharpa/right/command'])
@pytest.mark.parametrize('ms',[122,132,149,150,151,170,250])
@pytest.mark.parametrize('phase',[0,7,19,32])
def test_hand_gaps_against_real_converter(topic,ms,phase):
    raw=fixture();gap(raw,topic,ms,phase);compare(raw)


@pytest.mark.parametrize('topic',[t for t in TOPICS if '/tactile/' in t])
@pytest.mark.parametrize('ms',[140,151,300,500])
def test_every_tactile_channel_is_required(topic,ms):
    raw=fixture();gap(raw,topic,ms,19);compare(raw)


@pytest.mark.parametrize('ms',[150,151,170,250,300])
def test_depth_strict_without_cleaner_seam(ms):
    raw=fixture();gap(raw,'/cam0/depth/image_raw',ms,19);compare(raw)


@pytest.mark.parametrize('frames',[1,2,3])
@pytest.mark.parametrize('camera',[0,1,2])
def test_rgb_joint_short_gap_policy(camera,frames):
    raw=fixture();topic=f'/cam{camera}/color/image_raw'
    left=3*NS
    # First invalid output grid is at left+166.666ms.
    right=left+(5+frames)*NS//30
    raw[topic]=[(t,m) for t,m in raw[topic] if t<=left]+[(right,message(topic))]+[(t,m) for t,m in raw[topic] if t>right]
    quality,passed=compare(raw)
    assert passed is (frames<=2)
    assert quality.accepted_rgb_frames==(frames if frames<=2 else 0)


def test_staggered_rgb_union_cannot_bridge_three_frames():
    raw=fixture()
    for camera,offset in [(1,0),(2,1)]:
        left=3*NS+offset*NS//30;right=left+7*NS//30
        topic=f'/cam{camera}/color/image_raw'
        raw[topic]=[(t,m) for t,m in raw[topic] if t<=left or t>=right]
    quality,passed=compare(raw)
    assert not passed and quality.first_failure


@pytest.mark.parametrize('delay',[0,.05,.249,.4])
def test_cross_topic_late_delivery_cannot_reverse_confirmation(delay):
    raw=fixture()
    def delays(topic,t):return delay if topic=='/sharpa/right/joint_states' and 3*NS<t<3*NS+500_000_000 else 0
    # Preserve per-topic source order; a burst catches up after the delay.
    latest={}
    def ordered(topic,t):
        receipt=max(t/NS+delays(topic,t),latest.get(topic,0)+1e-9);latest[topic]=receipt
        return receipt-t/NS
    quality,passed=compare(raw,delays=ordered)
    assert passed and quality.first_failure is None


def test_held_arm_action_and_reengagement_transition():
    raw=fixture()
    raw[STATUS]=[(t,message(STATUS,() if 2*NS<t<4*NS else ('left','right'))) for t,m in raw[STATUS]]
    raw[ACTION]=[(t,m) for t,m in raw[ACTION] if t<=2*NS or t>=4*NS]
    compare(raw)
    raw[ACTION]=[(t,m) for t,m in raw[ACTION] if t<4*NS or t>4*NS+200_000_000]
    quality,passed=compare(raw)
    assert not passed


def test_only_trailing_invalid_is_trimmed_not_failed():
    raw=fixture();raw['/sharpa/left/joint_states']=[(t,m) for t,m in raw['/sharpa/left/joint_states'] if t<=4*NS]
    quality,passed=compare(raw)
    assert passed


def test_grid_ns_boundary_is_closed():
    policy=AlignmentPolicy(trim_start_sec=0,trim_end_sec=0)
    topic='/sharpa/left/joint_states'
    q=AlignmentQuality([topic],policy)
    q.observe(topic,message(topic),NS,0)
    assert not q._point(NS+150_000_000)['base_bad']
    assert q._point(NS+150_000_001)['base_bad']


def test_legacy_gap_is_only_diagnostic_in_b_but_timestamp_failure_stays():
    from teleop_data_collector.continuity import ContinuityMonitor
    topic='/sharpa/left/joint_states'
    q=ContinuityMonitor([topic],mode='conversion_aligned')
    q.begin('synthetic',0);q.observe_message(topic,message(topic),NS,0)
    q.observe_message(topic,message(topic),NS+170_000_000,.17)
    assert q.first_failure is None and q.snapshot()['source_warnings']
    q.observe_message(topic,message(topic),NS+170_000_000,.18)
    assert q.first_failure['category']=='timestamp'


@pytest.mark.parametrize('seam',[False,True])
def test_depth_seam_requires_known_boundary_and_accepted_rgb_overlap(seam):
    raw=fixture()
    for topic in ('/cam0/color/image_raw','/cam0/depth/image_raw'):
        gap(raw,topic,233,0)
    q,passed=compare(raw,boundaries=(3*NS+190_000_000,) if seam else ())
    assert passed is seam
    assert (q.accepted_depth_frames>0) is seam


@pytest.mark.parametrize('delta',[-1,0,1])
def test_rgb_age_250ms_boundary(delta):
    raw=fixture();gap(raw,'/cam1/color/image_raw',250,delta/1_000_000)
    compare(raw)


def test_metadata_capacity_is_explicit_failure_and_bounded():
    topic='/sharpa/left/joint_states';q=AlignmentQuality([topic])
    for i in range(5000):q.observe(topic,message(topic),NS+i,i/1e9)
    assert q.first_failure['category']=='alignment_capacity'
    assert len(q.samples[topic].times)==4096


@pytest.mark.parametrize('topic',['/sharpa/left/joint_states','/sharpa/right/command','/cam1/color/image_raw','/sharpa/right/tactile/pinky/deformation'])
def test_converter_accepted_low_mean_frequency_is_not_an_independent_failure(topic):
    raw=fixture();raw[topic]=[(NS+i*NS//8,message(topic)) for i in range(33)]
    q,passed=compare(raw)
    assert passed


def test_trailing_hole_with_a_later_sample_outside_trim_is_not_internal():
    raw=fixture();topic='/sharpa/left/joint_states'
    raw[topic]=[(t,m) for t,m in raw[topic] if t<=4*NS or t==5*NS]
    q,passed=compare(raw)
    assert passed and q.invalid_count>0


def test_start_trim_excludes_internal_raw_gap_before_selected_window():
    raw=fixture();topic='/sharpa/left/joint_states'
    raw[topic]=[(t,m) for t,m in raw[topic] if t<=NS+200_000_000 or t>=NS+800_000_000]
    q,passed=compare(raw)
    assert passed


def test_controller_only_requests_stop_for_grid_failure_not_old_gap():
    from unittest.mock import Mock
    from teleop_data_collector.continuity import ContinuityMonitor
    from teleop_data_collector.collector_control import CollectorController
    topic='/sharpa/left/joint_states'
    policy=AlignmentPolicy(trim_start_sec=0,trim_end_sec=0,arrival_settle_ms=0)
    monitor=ContinuityMonitor([topic],mode='conversion_aligned',alignment_policy=policy)
    monitor.begin('synthetic',0)
    controller=CollectorController(SimpleNamespace(continuity_monitor=monitor,get_logger=lambda:Mock()),SimpleNamespace())
    controller.request_stop=Mock()
    for t in (NS,NS+170_000_000):
        monitor.observe_message(topic,message(topic),t,t/NS)
        monitor.advance_alignment(t/NS)
    controller.check_continuity()
    controller.request_stop.assert_not_called()
    monitor.observe_message(topic,message(topic),NS+500_000_000,1.5)
    monitor.advance_alignment(1.5)
    controller.check_continuity()
    controller.request_stop.assert_called_once()
    reason=controller.request_stop.call_args.kwargs['reason']
    assert reason['category']=='alignment' and 'grid_index' in reason['alignment']


def test_bad_payload_and_status_refuse_independently_of_grid_quality():
    from teleop_data_collector.alignment_quality import validate_metadata
    with pytest.raises(ValueError):validate_metadata(STATUS,SimpleNamespace(accepted_sides=['left'],faults=['hardware fault']))
    with pytest.raises(ValueError):validate_metadata('/sharpa/left/tactile/index/wrench',SimpleNamespace(wrench=SimpleNamespace(force=SimpleNamespace(x=float('nan'),y=0,z=0),torque=SimpleNamespace(x=0,y=0,z=0))))



def test_sharpa_command_names_are_unprefixed_and_state_names_prefixed():
    from teleop_core.contract import SHARPA_JOINT_NAMES
    from teleop_data_collector.alignment_quality import validate_metadata
    command=SimpleNamespace(name=list(SHARPA_JOINT_NAMES),position=[0.]*22)
    validate_metadata('/sharpa/left/command',command)
    with pytest.raises(ValueError):validate_metadata('/sharpa/left/joint_states',command)
    state=SimpleNamespace(name=['left_'+n for n in SHARPA_JOINT_NAMES],position=[0.]*22,velocity=[0.]*22)
    validate_metadata('/sharpa/left/joint_states',state)


@pytest.mark.parametrize('topic',['/cam0/depth/image_raw',*[t for t in TOPICS if '/tactile/' in t]])
def test_extra_age_boundary_is_150ms_closed(topic):
    q=AlignmentQuality([topic]);q.observe(topic,message(topic),NS,0)
    assert not q._point(NS+150_000_000)['extra_bad']
    assert q._point(NS+150_000_001)['extra_bad']==[topic]



def test_hold_to_end_does_not_shrink_window_to_last_action():
    raw=fixture()
    raw[STATUS]=[(t,message(STATUS,() if t>2*NS else ('left','right'))) for t,m in raw[STATUS]]
    raw[ACTION]=[(t,m) for t,m in raw[ACTION] if t<=2*NS]
    q,passed=compare(raw)
    assert passed and q.index>80


def test_one_arm_action_cannot_cover_the_other_engaged_arm():
    raw=fixture()
    raw[STATUS]=[(t,message(STATUS,('left',) if 2*NS<t<3*NS else ('left','right'))) for t,m in raw[STATUS]]
    for t,m in raw[ACTION]:
        if 2*NS<t<4*NS:m.name=LEFT_COMMAND_JOINT_NAMES
    q,passed=compare(raw)
    assert not passed and any('#right' in k for k in q.first_failure['invalid_topics'])


def test_fractional_trim_uses_sidecar_integer_ns_window():
    q,passed=compare(fixture(),policy=AlignmentPolicy(trim_start_sec=.1234567899,trim_end_sec=.2345678919))
    assert passed and q.anchor==NS+123456789
