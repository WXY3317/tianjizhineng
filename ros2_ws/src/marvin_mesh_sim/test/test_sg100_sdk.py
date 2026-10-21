# 使用原版 SG100 库验证命令序列化、虚拟手掌处理和反馈解码。
"""Real vendor SO serialization, virtual palm and real SO feedback decoding."""
import json
from pathlib import Path
import struct
import threading
import time

import numpy as np
import pytest
import yaml

from marvin_sim.local_address import local_address
from marvin_mesh_sim.sg100_client import SG100Client
from marvin_mesh_sim.sg100_hands import SG100Hands
from marvin_mesh_sim.sg100_protocol import Stream,crc32,frame

ROOT=Path(__file__).resolve().parents[4]
PACKAGE=ROOT/'ros2_ws/src/marvin_mesh_sim'
SDK=ROOT/'SG100_SDK_Publish'


def test_sg100_names_follow_vendor_finger_map():
    config=yaml.safe_load((PACKAGE/'config/hands.yaml').read_text())
    for side in 'LR':
        assert [j['name'] for j in config[side]['joints']]==[f'{side}_Joint{i}' for i in range(1,12)]
        assert [j['sdk_id'] for j in config[side]['joints']]==list(range(1,12))
        assert [j['sdk_finger'] for j in config[side]['joints']]==[
            *['FINGER_THUMB']*3,*['FINGER_INDEX']*3,*['FINGER_MIDDLE']*2,*['FINGER_RING']*3]
        assert config[side]['joints'][8]['source_name']==f'{side}_little_j1'


def test_sg100_stream_crc_fragments_and_rejects_bad_frames():
    assert crc32(b'123456789')==0xfc891918  # CRC-32/BZIP2 published check value
    stream=Stream();raw=frame(0x26)
    assert stream.feed(raw[:3])==[]
    assert stream.feed(raw[3:])==[raw]
    bad=bytearray(raw);bad[-1]^=1
    with pytest.raises(ValueError,match='CRC'):stream.feed(bad)


def test_original_sg100_both_hands_wire_feedback_and_stale_detection():
    hands=SG100Hands(PACKAGE,local_address(),SDK,port=0,wire_port=0)
    done=threading.Event();errors=[];client=None
    def run():
        try:
            while not done.wait(.003):hands.tick()
        except BaseException as exc:errors.append(exc)
    thread=threading.Thread(target=run);thread.start()
    try:
        client=SG100Client(SDK,PACKAGE,port=hands.wire_port)
        deadline=time.monotonic()+2
        while True:
            try:client.read();break
            except RuntimeError:
                assert time.monotonic()<deadline
                time.sleep(.01)
        goal=np.array([.15,-.2,.1,-.1,.2,.1,.2,.1,.1,.2,.1])
        for side in 'LR':client.move(side,goal)
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            feedback=client.read()
            if all(np.max(np.abs(feedback[s]-goal))<1e-6 for s in 'LR'):break
            time.sleep(.01)
        assert not errors
        for side in 'LR':
            assert np.max(np.abs(feedback[side]-goal))<1e-6
            assert np.max(np.abs(hands.q[side]-goal))<1e-6
            assert client.devices[side].updates>5
            assert hands.palms[side].rejected==0,hands.palms[side].last_error
        before=hands.target['L'].copy()
        for bad in ([float('nan')]*11,[99.]*11,[0.]*10):
            with pytest.raises(ValueError):client.move('L',bad)
        # Invalid mode/ID/limit batches must not partially update the hand.
        mapping=json.loads((SDK/'joint_angle_map_v3.0.json').read_text())['joint_angle_map']['L']
        for invalid_id,invalid_mode,invalid_angle in ((1,8,0.),(0,1,0.),(1,1,999.)):
            data=b''.join(struct.pack('<BBfHHHH2x',invalid_id if i==0 else i+1,
                        invalid_mode if i==0 else 1,
                        invalid_angle if i==0 else float(mapping[i]['a']*before[i]),0,0,0,0)
                         for i in range(11))
            with pytest.raises(ValueError):hands.palms['L'].handle(frame(5,data,dst=0xfd,src=0xfe))
            assert np.array_equal(hands.target['L'],before)
        done.set();thread.join(timeout=2)
        time.sleep(.6)
        with pytest.raises(RuntimeError,match='stale'):client.read()
    finally:
        if client:client.close()
        done.set();thread.join(timeout=2);hands.close()
