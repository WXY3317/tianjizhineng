"""Bounded local IK recovery; no joint limits, target pose or path are relaxed."""
import math


MAX_STEP_DEG = 1.0  # Same discontinuity bound as the Cartesian planner.


def solve_step(engine, arm, matrix, reference):
    limits=engine.limits[arm]

    def attempt(seed):
        engine.check_cancel()
        params=engine.ik_type()
        for i,value in enumerate(seed):params.m_Input_IK_RefJoint[i]=value
        for i,value in enumerate(x for row in matrix for x in row):params.m_Input_IK_TargetTCP[i]=value
        result=engine.robot.inverse_kinematics(arm,params)
        engine.check_cancel()
        is_params=isinstance(result,engine.ik_type)
        limit_error=(not is_params and result==-1011) or (is_params and bool(params.m_Output_IsJntExd))
        if not is_params:return None,params,f'IK failed: {result}',limit_error
        if params.m_Output_IsOutRange:return None,params,'IK outside workspace',False
        if any(params.m_Output_IsDeg):return None,params,'IK is singular',False
        if params.m_Output_IsJntExd or any(params.m_Output_JntExdTags):
            return None,params,'IK exceeds joint limits',True
        q=list(params.m_Output_RetJoint)
        engine.h.finite(q)
        if any(not lo<=value<=hi for value,(lo,hi) in zip(q,limits)):
            return None,params,'IK exceeds measured joint limits',True
        actual=engine.h.fk(engine.robot,arm,q)
        position=math.sqrt(sum((actual[i][3]-matrix[i][3])**2 for i in range(3)))*engine.c.sdk_length_to_m
        rotation=max(abs(actual[i][j]-matrix[i][j]) for i in range(3) for j in range(3))
        if position>.0001 or rotation>.001:
            return None,params,f'IK residual too large: {position} m / {rotation}',False
        return q,params,None,False

    q,original,error,limit_error=attempt(reference)
    if q is not None:return q
    if not limit_error:raise ValueError(f'arm{arm} {error}')

    tried={tuple(reference)}
    attempts=1

    def search(seeds):
        nonlocal attempts
        candidates=[]
        for seed in seeds:
            if len(seed)!=7 or not all(math.isfinite(v) for v in seed):continue
            if any(not lo<=v<=hi for v,(lo,hi) in zip(seed,limits)):continue
            if max(abs(a-b) for a,b in zip(seed,reference))>MAX_STEP_DEG:continue
            key=tuple(seed)
            if key in tried:continue
            tried.add(key)
            attempts+=1
            candidate,_,_,_=attempt(seed)
            if candidate is not None and max(abs(a-b) for a,b in zip(candidate,reference))<=MAX_STEP_DEG:
                candidates.append(candidate)
        return min(candidates,key=lambda q:sum((a-b)**2 for a,b in zip(q,reference))) if candidates else None

    # Re-solve candidate seeds so SDK singularity/dynamic-limit checks apply to
    # the selected solution too. Raw table entries never become commands directly.
    recovered=[]
    count=int(original.m_OutPut_Result_Num)
    if 0<=count<=8:
        seeds=[list(original.m_OutPut_AllJoint[i*8:i*8+7]) for i in range(count)]
        q=search(seeds)
        if q is not None:recovered.append(q)
    # Smaller neighborhoods first reduce the risk of introducing rate discontinuities.
    # At most 8 SDK candidates plus 4*14 perturbed references are examined.
    for offset in (.001,.005,.025,.1):
        seeds=[]
        for joint in range(7):
            for sign in (-1,1):
                seed=list(reference);seed[joint]+=sign*offset;seeds.append(seed)
        q=search(seeds)
        if q is not None:recovered.append(q)
    if recovered:
        return min(recovered,key=lambda q:sum((a-b)**2 for a,b in zip(q,reference)))

    exceeded=[]
    for i,tag in enumerate(original.m_Output_JntExdTags):
        if not tag:continue
        lo,hi=limits[i]
        runtime_lo=float(original.m_Output_RunLmtN[i]);runtime_hi=float(original.m_Output_RunLmtP[i])
        if math.isfinite(runtime_lo) and math.isfinite(runtime_hi) and runtime_lo<runtime_hi:
            lo,hi=runtime_lo,runtime_hi
        exceeded.append(f'joint{i+1}={original.m_Output_RetJoint[i]:.4f} deg, limits=[{lo:.4f},{hi:.4f}]')
    detail='; '.join(exceeded) or 'SDK did not identify the exceeded joint'
    raise ValueError(f'arm{arm} {error}; {detail}; no continuous legal IK found in {attempts} local attempts')
