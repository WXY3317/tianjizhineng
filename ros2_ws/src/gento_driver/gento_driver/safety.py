"""Independent validation at the hardware boundary; no planner is trusted."""
import math
from .conversions import JOINT_NAMES, BODY_JOINT_NAMES, finite, pose_matrix

OBJECT_NAMES = {0: JOINT_NAMES[:7], 1: JOINT_NAMES[7:], 3: BODY_JOINT_NAMES}


def validate_goal(goal, snapshot, hardware, now_ns, check_freshness=True, planning_limits=None):
    if goal.session_id != snapshot['session_id']:
        raise ValueError('Stale driver session; obtain a new snapshot and replan')
    if not goal.task_id or len(goal.task_id) > 128:
        raise ValueError('Invalid task ID')
    state = goal.start_state
    age = (now_ns-state.header.stamp.sec*10**9-state.header.stamp.nanosec)/1e9
    if check_freshness and not 0 <= age <= hardware.c.command_max_age:
        raise ValueError('Stale or future trajectory start state')
    all_names = JOINT_NAMES+BODY_JOINT_NAMES
    if list(state.name) != all_names or len(state.position) != 20:
        raise ValueError('Expected complete start state with 20 named joints')
    finite(state.position)
    if state.velocity or state.effort:raise ValueError('Start state must contain positions only')
    start = [math.degrees(q) for q in state.position]
    actual = snapshot['joints'][0]+snapshot['joints'][1]+snapshot['body']
    if max(abs(x-y) for x,y in zip(start,actual)) > .2:
        raise ValueError('Robot moved since planning; replan required')
    if not 1 <= len(goal.trajectories) <= 2:
        raise ValueError('Expected one body trajectory or one/two arm trajectories')
    if len(goal.controller_ratios)!=2*len(goal.trajectories):raise ValueError('Missing expected controller ratios')
    finite(goal.controller_ratios)
    objects, plans = [], {}
    for trajectory in goal.trajectories:
        if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
            raise ValueError('Scheduled trajectory headers are unsupported; action starts immediately')
        obj = next((key for key,names in OBJECT_NAMES.items() if list(trajectory.joint_names)==names), None)
        if obj is None or obj in objects:
            raise ValueError('Unknown or duplicate trajectory joints')
        objects.append(obj)
        if obj == 3 and len(goal.trajectories) != 1:
            raise ValueError('Body and arms cannot be executed together')
        count = len(trajectory.points)
        if (obj == 3 and count != 1) or (obj != 3 and not 5 <= count <= hardware.robot.max_point):
            raise ValueError('Invalid trajectory point count')
        points = []
        for index,point in enumerate(trajectory.points):
            if len(point.positions) != len(OBJECT_NAMES[obj]) or point.velocities or point.accelerations or point.effort:
                raise ValueError('Only complete joint position samples are accepted')
            finite(point.positions)
            if point.time_from_start.sec*10**9+point.time_from_start.nanosec != index*20000000:
                raise ValueError('Trajectory samples must be exactly 20 ms apart, starting at zero')
            points.append([math.degrees(q) for q in point.positions])
        plans[obj] = points
    # After enabling SDK playback, only the driver's previously measured low-speed
    # limits may be reused. Limits are never supplied by the action client.
    arm_limits = (hardware.read_cartesian_rate_limits([a for a in objects if a != 3])
                  if planning_limits is None else planning_limits)
    for index,obj in enumerate(objects):
        settings=arm_limits[obj] if obj!=3 else snapshot['controller']['body']['set']
        finite([settings['vel_ratio'],settings['acc_ratio']])
        if not all(0<settings[k]<=100 for k in ('vel_ratio','acc_ratio')):raise ValueError('Invalid controller ratios')
        expected=goal.controller_ratios[index*2:index*2+2]
        if any(abs(a-settings[key])>1e-6 for a,key in zip(expected,('vel_ratio','acc_ratio'))):
            raise ValueError('Controller ratios changed since planning; replan required')
    max_duration = 0.
    for obj,points in plans.items():
        origin = start[obj*7:obj*7+7] if obj != 3 else start[14:]
        limits = hardware.limits[obj] if obj != 3 else hardware.body_limits
        for q in points:
            for i,(value,initial,(lo,hi)) in enumerate(zip(q,origin,limits)):
                if obj==3:
                    # ROS radians -> SDK degrees may overshoot a limit by an
                    # ULP. Tolerate only conversion noise, then clamp the actual
                    # command back into exact controller limits before SDK use.
                    epsilon=1e-10  # degrees, far below encoder resolution
                    if not lo-epsilon<=value<=hi+epsilon or abs(value-initial)>hardware.c.max_delta+epsilon:
                        raise ValueError(f'object{obj} joint{i+1} exceeds limits/max_delta')
                    q[i]=min(hi,max(lo,value))
                    continue
                if not lo <= value <= hi or abs(value-initial)>hardware.c.max_delta:
                    raise ValueError(f'object{obj} joint{i+1} exceeds limits/max_delta')
        if obj == 3: continue
        caps = arm_limits[obj]
        entry = [(x-y)*50 for x,y in zip(points[0],actual[obj*7:obj*7+7])]
        previous = entry
        for index in range(len(points)):
            velocity = entry if index == 0 else [(x-y)*50 for x,y in zip(points[index],points[index-1])]
            acceleration = [v*50 for v in velocity] if index == 0 else [(v-u)*50 for v,u in zip(velocity,previous)]
            if any(abs(v)>cap+1e-7 for v,cap in zip(velocity,caps['speed'])):
                raise ValueError(f'object{obj} speed limit exceeded at sample {index}')
            if any(abs(a)>cap+1e-7 for a,cap in zip(acceleration,caps['acceleration'])):
                raise ValueError(f'object{obj} acceleration limit exceeded at sample {index}')
            previous = velocity
        if any(abs(v)*50>cap+1e-7 for v,cap in zip(previous,caps['acceleration'])):
            raise ValueError('Terminal deceleration exceeds limits')
        max_duration = max(max_duration,(len(points)-1)/50)
    if max_duration+1 >= hardware.c.timeout:
        raise ValueError('Trajectory exceeds motion timeout')
    lines = {}
    if not len(goal.line_objects)==len(goal.line_starts)==len(goal.line_goals):
        raise ValueError('Cartesian line metadata lengths differ')
    for obj,a,b in zip(goal.line_objects,goal.line_starts,goal.line_goals):
        if obj not in plans or obj == 3 or obj in lines:
            raise ValueError('Invalid Cartesian line object')
        begin,end = pose_matrix(a,hardware.c.sdk_length_to_m),pose_matrix(b,hardware.c.sdk_length_to_m)
        for index in (0,len(plans[obj])-1):
            fk = hardware.h.fk(hardware.robot,obj,plans[obj][index])
            target = begin if index == 0 else end
            if (math.sqrt(sum((fk[i][3]-target[i][3])**2 for i in range(3)))*hardware.c.sdk_length_to_m > .0001 or
                    max(abs(fk[i][j]-target[i][j]) for i in range(3) for j in range(3)) > .001):
                raise ValueError('Cartesian line endpoints disagree with joint trajectory')
        # Verify all planned points against the supplied segment before touching hardware.
        for q in plans[obj]:
            hardware.check_cancel()
            fk = hardware.h.fk(hardware.robot,obj,q)
            if line_error(fk,begin,end,hardware.c.sdk_length_to_m)[0] > .0002:
                raise ValueError('Planned FK departs the Cartesian segment')
        lines[obj]=(begin,end)
    return plans,lines


def line_error(actual,begin,end,scale):
    direction=[end[i][3]-begin[i][3] for i in range(3)]
    offset=[actual[i][3]-begin[i][3] for i in range(3)]
    length2=sum(x*x for x in direction)
    fraction=min(1.,max(0.,sum(x*y for x,y in zip(offset,direction))/length2)) if length2>1e-12 else 0.
    vector=[(x-fraction*y)*scale for x,y in zip(offset,direction)]
    return math.sqrt(sum(x*x for x in vector)),vector
