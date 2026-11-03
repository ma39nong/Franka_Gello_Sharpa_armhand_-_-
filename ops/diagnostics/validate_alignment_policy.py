#!/usr/bin/env python3
"""Run with the converter venv Python. Isolated synthetic CDR/full-job oracle.

No original bag/sidecar edits, network, SDK or production conversion submission.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tarfile
import tempfile

PIN = 'f5df024a40088bfdd73b44e9e155f6b5968af296'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--oracle-root',type=Path,required=True)
    parser.add_argument('--work-dir',type=Path)
    args=parser.parse_args()
    project=Path(__file__).resolve().parents[2]
    root=args.oracle_root.resolve()
    sha=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    if sha!=PIN:raise SystemExit(f'Oracle SHA mismatch: {sha}, expected {PIN}')
    subprocess.run(['git','-C',str(root),'diff','--exit-code','HEAD','--','src','configs','schemas'],check=True)
    work=args.work_dir.resolve() if args.work_dir else Path(tempfile.mkdtemp(prefix='sharpa-b-oracle-'))
    work.mkdir(parents=True,exist_ok=True)
    if any(work.iterdir()):raise SystemExit('Test work directory must be empty')
    archive=work/'converter';archive.mkdir()
    contents=subprocess.check_output(['git','-C',str(root),'archive',PIN])
    with tarfile.open(fileobj=io.BytesIO(contents)) as tar:tar.extractall(archive,filter='data')
    sys.path[:0]=[str(archive/'src'),str(archive/'tests')]
    import numpy as np
    import yaml
    from rosbags.typesys import Stores,get_typestore,get_types_from_msg
    from conversion_helpers import make_conversion_bag
    from convest.config import load_config
    from convest.sources import robot_rosbag2 as source
    from convest.recipes.multimodal import prepare,output_segments
    from convest.quality import RecordingRejected
    from convest.jobs import run,check_ready
    cfg=load_config(archive/'configs/sharpa.yaml')
    config_hash=hashlib.sha256((archive/'configs/sharpa.yaml').read_bytes()).hexdigest()
    store=get_typestore(Stores.ROS2_HUMBLE)
    for name in ('ArmCommandStatus','HandTelemetryStatus'):
        store.register(get_types_from_msg((archive/f'schemas/{name}.msg').read_text(),f'teleop_interfaces/msg/{name}'))
    cases=[('normal',None,0,0),('hand132','/sharpa/left/joint_states',132,0),
           ('hand170_phase19','/sharpa/left/joint_states',170,19),
           ('arm170_phase19','/teleop/validated_arm_commands',170,19),
           ('hand8hz','/sharpa/left/joint_states',0,0),
           ('camera10hz','/cam1/color/image_raw',0,0),
           ('hand250','/sharpa/right/joint_states',250,19),
           ('tactile300','/sharpa/left/tactile/index/wrench',300,19),
           ('depth300','/cam0/depth/image_raw',300,19),
           ('rgb3','/cam1/color/image_raw',267,0),
           ('idle_future_cut','/sharpa/left/joint_states',300,19)]
    for case_index,(name,topic,ms,phase) in enumerate(cases):
        bag=make_conversion_bag(work/'fixtures'/f'episode{case_index}','sharpa',frames=90)
        with sqlite3.connect(bag/'data.db3') as db:
            # Fixture images are synthetic. Match the existing collector's
            # required packed head depth shape before comparing policies.
            tid=db.execute("SELECT id FROM topics WHERE name='/cam0/depth/image_raw'").fetchone()[0]
            for row,payload in db.execute('SELECT id,data FROM messages WHERE topic_id=?',(tid,)).fetchall():
                kind='sensor_msgs/msg/Image';m=store.deserialize_cdr(payload,kind)
                m.width=640;m.height=400;m.step=1280;m.data=np.resize(m.data,512000).copy()
                db.execute('UPDATE messages SET data=? WHERE id=?',(bytes(store.serialize_cdr(m,kind)),row))
            if topic:
                tid,kind=db.execute('SELECT id,type FROM topics WHERE name=?',(topic,)).fetchone()
                if name in ('hand8hz','camera10hz'):
                    rows=db.execute('SELECT id FROM messages WHERE topic_id=? ORDER BY timestamp,id',(tid,)).fetchall()
                    stride=4 if name=='hand8hz' else 3
                    db.executemany('DELETE FROM messages WHERE id=?',[r for i,r in enumerate(rows) if i%stride])
                left=3_000_000_000+phase*1_000_000;right=left+ms*1_000_000
                payload=db.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 1',(tid,left+100)).fetchone()[0]
                db.execute('DELETE FROM messages WHERE topic_id=? AND timestamp>? AND timestamp<?',(tid,left+100,right+100))
                for stamp in (() if name in ('hand8hz','camera10hz') else (left,right)):
                    if db.execute('SELECT count(*) FROM messages WHERE topic_id=? AND timestamp=?',(tid,stamp+100)).fetchone()[0]:continue
                    m=store.deserialize_cdr(payload,kind);m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9)
                    db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(tid,stamp+100,bytes(store.serialize_cdr(m,kind))))
            if name=='idle_future_cut':
                for side in ('left','right'):
                    arm_id,kind=db.execute('SELECT id,type FROM topics WHERE name=?',(f'/{side}/franka/joint_states',)).fetchone()
                    for row,payload in db.execute('SELECT id,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ?',(arm_id,2_500_000_000,3_700_000_100)).fetchall():
                        m=store.deserialize_cdr(payload,kind);m.position=np.full(7,9.,dtype=np.float64)
                        db.execute('UPDATE messages SET data=? WHERE id=?',(bytes(store.serialize_cdr(m,kind)),row))
            meta=yaml.safe_load((bag/'metadata.yaml').read_text());info=meta['rosbag2_bagfile_information']
            for item in info['topics_with_message_count']:
                item['message_count']=db.execute('SELECT count(*) FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=?',(item['topic_metadata']['name'],)).fetchone()[0]
            info['message_count']=sum(t['message_count'] for t in info['topics_with_message_count'])
            info['files']=[{'path':'data.db3','starting_time':info['starting_time'],'duration':info['duration'],'message_count':info['message_count']}]
            (bag/'metadata.yaml').write_text(yaml.safe_dump(meta))
    # Real ROS bag postflight and actual recorder sidecar writer, isolated in Docker.
    inspect_code=r'''
import json,yaml
from pathlib import Path
from types import SimpleNamespace
from teleop_data_collector.bag_validation import inspect_bag
from teleop_data_collector.alignment_quality import AlignmentPolicy
from teleop_data_collector.collector_contract import recording_outcome
from teleop_data_collector.rosbag_recording_node import RosbagEpisodeRecorder
raw=yaml.safe_load(Path('/workspace/franka_upper_body_teleop/data_collection/config/record_gello_sharpa_tactile.yaml').read_text())['teleop_data_collector']['ros__parameters']
contracts=[SimpleNamespace(name=k,topic=v['topic'],type_name=v['type'],required=v.get('required',True),min_frequency_hz=v.get('min_frequency_hz',1),max_gap_ms=v.get('max_gap_ms',150)) for k,v in raw['topics'].items()]
results={}
for bag in sorted(Path('/fixtures').iterdir()):
 old,old_report=inspect_bag(bag,contracts,trim_start_sec=1,trim_end_sec=.2,continuity_enabled=True)
 failures,report=inspect_bag(bag,contracts,trim_start_sec=1,trim_end_sec=.2,continuity_enabled=True,continuity_mode='conversion_aligned',alignment_quality=AlignmentPolicy())
 outcome=recording_outcome(exit_code=0,stream_failures=failures)
 recorder=RosbagEpisodeRecorder(SimpleNamespace(),bag.parent,'synthetic',(),30,(),provenance={'test_only':True,'continuity_enabled':True,'continuity_mode':'conversion_aligned','alignment_quality':AlignmentPolicy().report(),'trim_start_sec':1.,'trim_end_sec':.2})
 recorder.set_live_continuity(report['continuity'])
 recorder._write_state(bag,state=outcome.state,finalized=outcome.finalized,failures=list(failures),validation_report=report)
 results[bag.name]={'old_stop':bool(old),'old_reasons':list(old),'B_stop':bool(failures),'first_reason':report['continuity'].get('auto_stop_reason'),'postflight_result':list(failures)}
Path('/fixtures/postflight.json').write_text(json.dumps(results,indent=2))
'''
    docker=['docker','run','--rm','-i','--network','none','-v',f'{project}:/workspace/franka_upper_body_teleop:ro','-v',f'{work/"fixtures"}:/fixtures',
            '-e','PYTHONPATH=/workspace/franka_upper_body_teleop/ros_ws/src/teleop_data_collector:/workspace/franka_upper_body_teleop/ros_ws/src/teleop_core','franka-upper-body-teleop:latest','python3','-']
    subprocess.run(docker,input=inspect_code,text=True,check=True)
    raw_results=json.loads((work/'fixtures/postflight.json').read_text())
    results={name:raw_results[f'episode{i}'] for i,(name,_,_,_) in enumerate(cases)}
    eligible=[]
    for case_index,(name,_,_,_) in enumerate(cases):
        bag=work/'fixtures'/f'episode{case_index}'
        item=source.discover(bag)[0]
        with source.Bag(bag,cfg['schema_dir']) as handle:
            _,aligned=prepare(source,handle,item,cfg,yaml.safe_load(Path(cfg['contract']).read_text()))
            try:output_segments(item,aligned,cfg);passed=True
            except RecordingRejected:passed=False
        results[name].update(align_result=passed,passes_conversion_entry=item['eligible'],is_production_eligible=False,test_only=True,
                             actual_conversion_result='not eligible',rgb_tolerance=aligned.report['camera_short_gap_tolerance'],depth_tolerance=aligned.report['depth_splice_tolerance'])
        assert passed is (not results[name]['B_stop']), (name,results[name])
        if item['eligible']:
            check_ready(bag,yaml.safe_load(Path(cfg['contract']).read_text()),cfg['schema_dir'])
            eligible.append(f'episode{case_index}')
    cfg.update(source_root=str(work/'fixtures'),min_free_gb=0)
    outcomes=run(cfg,archive/'outputs/offline-b',episodes=eligible,workers=1)
    # jobs.run returns a summary; journals are the authoritative per-episode result.
    for case_index,(name,_,_,_) in enumerate(cases):
        if f'episode{case_index}' not in eligible:continue
        job=json.loads((archive/'outputs/offline-b/jobs'/f'episode{case_index}.json').read_text())
        results[name]['actual_conversion_result']=job['state']
        results[name]['verification']=job.get('verification')
        assert job['state']=='completed',(name,job)
    # Explicit counterfactual qualification, isolated copy only: test whether
    # future cleaner cuts can remove an online-confirmed raw internal defect.
    import shutil
    original=work/'fixtures'/f'episode{len(cases)-1}'
    before=(original/'collection_state.json').read_bytes()
    copy=work/'test_only_counterfactual/episode0';shutil.copytree(original,copy)
    state=json.loads((copy/'collection_state.json').read_text())
    assert state['finalized'] is False and state['failures']
    state.update(state='finalized',finalized=True,failures=[],auto_stop_reason=None,
                 test_only=True,test_only_qualification='counterfactual temporal comparison; original failure unchanged')
    (copy/'collection_state.json').write_text(json.dumps(state))
    countercfg={**cfg,'source_root':str(copy.parent)}
    run(countercfg,archive/'outputs/counterfactual',episodes=['episode0'],workers=1)
    journal=json.loads((archive/'outputs/counterfactual/jobs/episode0.json').read_text())
    assert journal['state']=='completed',journal
    assert (original/'collection_state.json').read_bytes()==before
    results['idle_future_cut']['test_only_after_future_cleaner']=journal['state']
    results['idle_future_cut']['original_failure_preserved']=True
    report={'oracle_sha':sha,'oracle_config_sha256':config_hash,'test_only':True,'work_dir':str(work),'results':results,
            'reduced_stops':sum(r['old_stop'] and not r['B_stop'] for r in results.values())}
    (work/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
