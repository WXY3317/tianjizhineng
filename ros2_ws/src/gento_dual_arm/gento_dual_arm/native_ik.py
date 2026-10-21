"""Match the native Gento IK ABI without changing vendor sources."""
import ctypes


def bind_native_ik(robot, sdk):
    # FXType.h defines FX_BOOL as unsigned char, not the wrapper's c_int.
    arrays = {'m_Output_IsDeg', 'm_Output_JntExdTags'}
    flags = {'m_Output_IsOutRange', 'm_Output_IsJntExd'}
    native = type('NativeIK', (ctypes.Structure,), {'_fields_': [
        (name, ctypes.c_ubyte * 7 if name in arrays else
         ctypes.c_ubyte if name in flags else kind)
        for name, kind in sdk.FX_InvKineSolverParams._fields_
    ]})
    robot.dll.FX_L1_Kinematics_InverseKinematics.argtypes = [
        ctypes.c_void_p, ctypes.c_int32, ctypes.POINTER(native)]
    return native
