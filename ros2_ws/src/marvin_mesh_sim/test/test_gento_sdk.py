# 使用原版 Gento 动态库验证本地协议、躯干头部控制及 ctypes 结构体布局。
"""Exercise the vendor Python wrapper and native library against local controller."""
from pathlib import Path
import struct
import threading
import time

import numpy as np
import pytest

from marvin_sim.local_address import local_address
from marvin_mesh_sim.body import Body
from marvin_mesh_sim.gento_client import GentoBodyClient
from marvin_mesh_sim.gento_protocol import GentoController,decode

ROOT=Path(__file__).resolve().parents[4]
PACKAGE=ROOT/'ros2_ws/src/marvin_mesh_sim'


def packet(*commands):
    payload=b''
    for code,data in commands:
        command=bytes([code])+struct.pack('<H',len(data))+data
        payload+=command+bytes([-sum(command)%256])
    payload=bytes([len(commands)])+payload
    return b'Fa'+struct.pack('<H',len(payload)-1)+bytes([sum(payload)%256])+payload


def test_protocol_rejects_invalid_batches_atomically():
    body=Body(PACKAGE)
    server=GentoController(body,ROOT/'tianji-robot-SDK',local_address(''))
    try:
        before=body.target.copy()
        with pytest.raises(ValueError,match='channel 3'):
            server.handle(packet((90,struct.pack('<h',1)),(91,struct.pack('<6f',0,2,0,0,0,0)),
                                 (60,struct.pack('<h',1)),(61,struct.pack('<3f',0,0,1))))
        assert np.array_equal(body.target,before)
        assert server.modes=={60:0,90:0}
        bad=bytearray(packet((60,struct.pack('<h',1))));bad[-1]^=1
        with pytest.raises(ValueError):decode(bad)
        with pytest.raises(ValueError,match='Unsupported'):
            server.handle(packet((123,b'')))
    finally:server.close()


def test_original_native_sdk_body_head_feedback_and_stop():
    body=Body(PACKAGE)
    server=GentoController(body,ROOT/'tianji-robot-SDK',local_address(''))
    done=threading.Event();errors=[];client=None
    def tick():
        try:
            while not done.is_set():
                body.tick();server.tick();done.wait(.005)
        except BaseException as exc:errors.append(exc)
    thread=threading.Thread(target=tick);thread.start()
    try:
        client=GentoBodyClient(ROOT/'tianji-robot-SDK',PACKAGE)
        goal=client.read();goal[1]=.04;goal[6]=.05
        client.move([body.names[1],body.names[6]],[.04,.05])
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if np.max(np.abs(client.read()-goal))<1e-6:break
            time.sleep(.02)
        assert not errors
        assert np.max(np.abs(client.read()-goal))<1e-6
        assert server.received>5 and server.rejected==0,server.last_error
        client.stop()
        assert server.modes=={60:0,90:0}
        with pytest.raises(ValueError):client.move([body.names[6]],[999.])
    finally:
        if client:client.close()
        done.set();thread.join(timeout=2);server.close()


def test_gento_ik_abi_matches_compiled_vendor_header(tmp_path):
    """The shipped Python wrapper uses int for C unsigned-char IK flags."""
    import ctypes
    import subprocess
    from marvin_mesh_sim.gento_kinematics import GentoMeshKinematics
    kine=GentoMeshKinematics(ROOT/'tianji-robot-SDK',PACKAGE)
    fields=['m_Output_IsOutRange','m_Output_IsDeg','m_Output_JntExdTags',
            'm_Output_JntExdABS','m_Output_IsJntExd','m_Output_RunLmtP','m_Output_RunLmtN']
    code='#include <iostream>\n#include <cstddef>\n#include "FXKinematics.h"\nint main(){\n'
    code+='std::cout << sizeof(FX_InvKineSolvePara);\n'
    for field in fields:code+=f'std::cout << " " << offsetof(FX_InvKineSolvePara,{field});\n'
    code+='}\n'
    source=tmp_path/'abi.cpp';source.write_text(code)
    includes=sorted({p.parent for p in (ROOT/'tianji-robot-SDK/C_SDK').rglob('*.h')})
    subprocess.run(['g++','-std=c++11',str(source),*[f'-I{p}' for p in includes],'-o',str(tmp_path/'abi')],check=True,capture_output=True)
    actual=list(map(int,subprocess.check_output([str(tmp_path/'abi')],text=True).split()))
    expected=[ctypes.sizeof(kine.IK)]+[getattr(kine.IK,f).offset for f in fields]
    assert actual==expected
