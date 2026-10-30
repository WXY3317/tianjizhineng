"""Feedback-owned motion records. No hardware calls or control imports."""
import json
import os
from pathlib import Path
import tempfile
import time


def save_record(directory,record):
    root=Path(directory).expanduser()
    root.mkdir(parents=True,exist_ok=True)
    path=root/(str(record['created_ns'])+'-'+record['plan']['task_id']+'.json')
    for sample in record['samples']:
        sample['joint_matching']={}
        for obj,path_points in record['plan']['plans'].items():
            q=sample['body'] if obj=='3' else sample['joints'][int(obj)]
            nearest=min(range(len(path_points)),key=lambda i:sum((a-b)**2 for a,b in zip(q,path_points[i])))
            sample['joint_matching'][obj]={'nearest_index':nearest,
                'error_deg':[a-b for a,b in zip(q,path_points[nearest])]}
            tracking=sample.get('tracking',[])
            if obj!='3' and len(tracking)>int(obj) and tracking[int(obj)].get('available'):
                command=tracking[int(obj)]['command_joints_deg']
                match=min(range(len(path_points)),key=lambda i:sum((a-b)**2 for a,b in zip(command,path_points[i])))
                sample['joint_matching'][obj]['command_nearest_index']=match
                sample['joint_matching'][obj]['command_error_deg']=[a-b for a,b in zip(command,path_points[match])]
    fd,tmp=tempfile.mkstemp(prefix='.trajectory-',dir=root)
    try:
        with os.fdopen(fd,'w') as stream: json.dump(record,stream,ensure_ascii=False,allow_nan=False)
        os.replace(tmp,path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    return str(path)
