"""Small read/validation primitives for the sole SDK owner. No motion planner."""
import math


def check(result,operation):
    if result!=0:raise RuntimeError(f'{operation} failed: SDK={result}')


def finite(values):
    if not all(math.isfinite(v) for v in values):raise ValueError('Non-finite numeric input')


def fk(robot,arm,joints):
    matrix=robot.forward_kinematics(arm,joints)
    if not isinstance(matrix,list) or len(matrix)!=4 or any(len(row)!=4 for row in matrix):
        raise ValueError(f'arm{arm} FK failed: {matrix}')
    finite([v for row in matrix for v in row])
    return matrix


def read_limits(robot,arm,checkpoint=None):
    limits=[]
    for joint in range(7):
        pair=[]
        for suffix in ('LimitNeg','LimitPos'):
            if checkpoint:checkpoint()
            ret,value=robot.param_get_float(f'R.A{arm}.L{joint}.BASIC.{suffix}')
            if checkpoint:checkpoint()
            check(ret,f'arm{arm} joint{joint+1} {suffix}')
            pair.append(value)
        finite(pair)
        if pair[0]>=pair[1]:raise ValueError('Invalid controller joint limits')
        limits.append(pair)
    return limits


def tracking_sample(robot,arm,feedback,actual,with_fk=False):
    """Read controller-reported commands from the same RT frame as actual joints.

    Optional diagnostic fields must not replace the measured positions used by safety.
    The input command mailbox is deliberately not used: it may be stale in RunTraj.
    """
    try:
        command=list(feedback['cmd_pos'])
        if len(command)!=7:raise ValueError('Expected seven controller command positions')
        finite(command)
        velocity=feedback.get('fb_vel')
        if velocity is not None:
            velocity=list(velocity)
            if len(velocity)!=7:raise ValueError('Expected seven measured velocities')
            finite(velocity)
        return {'available':True,'command_joints_deg':command,'velocity_deg_s':velocity,
                'following_error_deg':[a-c for a,c in zip(actual,command)],
                'command_matrix':fk(robot,arm,command) if with_fk else None}
    except (KeyError,TypeError,ValueError,RuntimeError) as exc:
        return {'available':False,'reason':str(exc)}
