# 调用原版 Gento 原生库进行肩部到法兰的正逆解，并检查限位和正解回代误差。
# 单个参考关节解算失败时尝试有限组参考值；这里不负责 J4 和手掌 TCP 的坐标转换。
"""Mesh-calibrated FK/IK using only the Gento SDK's native kinematics handle."""
import ctypes
from pathlib import Path

import numpy as np
import yaml

from marvin_sim.kinematics import Kinematics,pose_error,validate_transform,INITIAL_DEG
from .gento_protocol import vendor


class GentoMeshKinematics(Kinematics):
    def __init__(self,sdk_root,model_root):
        self.sdk_root=Path(sdk_root)
        self.module=vendor(self.sdk_root)
        self.robot=self.module.GentoRobot(precision=7)
        self.robot.kine_log_level(0)
        # Vendor Python declares FX_BOOL as c_int in IK outputs, whereas its
        # C header defines unsigned char. Correct the ABI in this adapter only.
        # 按 C 头文件使用单字节布尔字段，修正 Python 包装的结构体布局；只修改本适配层，不改原厂文件。
        self.IK=type('NativeIK',(ctypes.Structure,),{'_fields_':[
            (name,ctypes.c_ubyte*7 if name in ('m_Output_IsDeg','m_Output_JntExdTags') else ctypes.c_ubyte
             if name in ('m_Output_IsOutRange','m_Output_IsJntExd') else kind)
            for name,kind in self.module.FX_InvKineSolverParams._fields_]})
        self.robot.dll.FX_L1_Kinematics_InverseKinematics.argtypes=[
            ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(self.IK)]
        model=Path(model_root)
        lines=[[float(v) for v in row.strip().split(',') if v.strip()]
               for row in (model/'config/mesh_arms.MvKDCfg').read_text().splitlines() if row.strip()]
        self.lower=[];self.upper=[];self.velocity=[];self.acceleration=[]
        geometry=yaml.safe_load((model/'config/sdk_geometry.yaml').read_text())
        limits=yaml.safe_load((model/'config/joint_limits.yaml').read_text())
        for arm in range(2):
            block=lines[arm*13:(arm+1)*13]
            rows=block[1:9];dh=[r[:4] for r in rows]
            pnva=[[r[5],r[4],r[6],r[7]] for r in rows[:7]]+[[0.]*4]
            bounds=block[9:13]
            result=self.robot.init_single_arm_manual(arm,int(block[0][0]),dh,pnva,bounds,
                         block[0][1:4],[0.]*7,[[0.]*3 for _ in range(7)],[[0.]*6 for _ in range(7)])
            if result!=0:raise RuntimeError(f'Gento SDK arm {arm} kinematics init failed: {result}')
            if self.robot.set_tool(arm,geometry['flange_tool_rotation'])!=0:
                raise RuntimeError('Gento SDK flange alignment failed')
            for j,row in enumerate(rows[:7]):
                bound=limits[f'joint{j+1}']['limit']
                self.lower.append(max(bound['lower'],np.radians(row[4])))
                self.upper.append(min(bound['upper'],np.radians(row[5])))
                self.velocity.append(min(bound['velocity'],np.radians(row[6])))
                self.acceleration.append(np.radians(row[7]))
        self.lower=np.array(self.lower);self.upper=np.array(self.upper)
        self.velocity=np.array(self.velocity);self.acceleration=np.array(self.acceleration)

    def fk(self,arm,joints):
        q=self.validate_joints(arm,joints)
        result=self.robot.forward_kinematics(arm,np.degrees(q).tolist())
        if isinstance(result,int):raise ValueError(f'Gento SDK FK failed: {result}')
        # 原版 SDK 输出平移为毫米；转换成米后再交给 ROS 和坐标变换层。
        value=validate_transform(result).copy();value[:3,3]*=.001
        return value

    def _ik_once(self,arm,target,reference):
        # 逆解边界进行反向单位转换：目标米→毫米，参考关节弧度→度。
        native=target.copy();native[:3,3]*=1000.
        request=self.IK()
        request.m_Input_IK_TargetTCP=(ctypes.c_double*16)(*native.ravel())
        request.m_Input_IK_RefJoint=(ctypes.c_double*7)(*np.degrees(reference))
        request.m_Input_IK_ZSPType=0
        result=self.robot.inverse_kinematics(arm,request)
        if isinstance(result,int):
            detail=' (candidate exceeds joint limits)' if result==-1011 else ''
            raise ValueError(f'Gento SDK IK code {result}{detail}')
        if result.m_Output_IsOutRange:raise ValueError('Gento SDK IK candidate outside workspace')
        if result.m_Output_IsJntExd:raise ValueError('Gento SDK IK candidate exceeds joint limits')
        if any(result.m_Output_IsDeg):raise ValueError('Gento SDK IK candidate is singular')
        q=self.validate_joints(arm,np.radians(list(result.m_Output_RetJoint)))
        # 对候选解做正解回代，防止 SDK 返回标志正常但几何误差过大的关节结果。
        dp,dr=pose_error(self.fk(arm,q),target)
        if dp>1e-5 or dr>1e-4:raise ValueError(f'Gento SDK IK validation failed: {dp:g} m, {dr:g} rad')
        return q

    def ik(self,arm,target,reference):
        target=validate_transform(target);reference=self.validate_joints(arm,reference)
        self.last_ik_info={'fallback':False,'attempts':1}
        try:return self._ik_once(arm,target,reference)
        except ValueError as exc:initial_error=str(exc)
        # A seven-axis IK failure for one reference does not prove that the
        # target is unreachable. Keep the target fixed and try bounded,
        # deterministic alternate references using the same original SDK.
        # 七自由度的解依赖参考姿态；目标和限位保持不变，有限尝试回到初始姿态及扰动参考值。
        section=slice(arm*7,arm*7+7);lower=self.lower[section];upper=self.upper[section]
        home=np.radians(INITIAL_DEG[arm])
        seeds=[reference+(home-reference)*fraction for fraction in (.25,.5,.75,1.)]
        seeds.extend([np.clip(np.zeros(7),lower,upper),(lower+upper)/2])
        for joint in (0,2):
            for degrees in (-90.,-60.,-30.,30.,60.,90.):
                seed=reference.copy();seed[joint]+=np.radians(degrees)
                seeds.append(np.clip(seed,lower,upper))
        solutions=[];attempts=1;seen=[reference]
        for seed in seeds:
            if any(np.allclose(seed,previous,atol=1e-10,rtol=0) for previous in seen):continue
            seen.append(seed);attempts+=1
            try:solutions.append(self._ik_once(arm,target,seed))
            except ValueError:continue
        self.last_ik_info={'fallback':True,'attempts':attempts,'valid_candidates':len(solutions),
                           'initial_error':initial_error}
        if not solutions:
            raise ValueError(f'Gento SDK IK: no validated solution after {attempts} references; first: {initial_error}')
        # Compare bounded joint displacement, without wrapping through limits.
        # 按相对原始参考关节的位移选择候选解，不能跨越关节限位做角度环绕。
        result=min(solutions,key=lambda q:float(np.linalg.norm(q-reference)))
        self.last_ik_info['max_joint_change_deg']=float(np.max(np.abs(np.degrees(result-reference))))
        return result
