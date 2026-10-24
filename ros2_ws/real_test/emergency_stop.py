"""连接控制器后立即请求全部对象软件急停，不复位或恢复运动。"""
import sys
from pathlib import Path

CONTROLLER_IP = '6.6.7.100'


def stop_all(robot, ip, mask):
    """返回 SDK 确认的对象掩码；始终释放连接和 SDK 资源。"""
    try:
        ret = robot.link(*map(int, str(ip).split('.')))
        if ret < 0:
            raise RuntimeError(f'连接失败：{ret}；急停未发送')
        return robot.emergency_stop(mask)
    finally:
        try:
            robot.cleanup()
        finally:
            robot.unlink()


def main():
    try:
        root = Path(__file__).resolve().parents[2] / 'tianji-robot-SDK'
        sys.path.insert(0, str(root))
        from PYTHON_SDK.GentoRobot import GentoRobot, FXObjMask

        mask = FXObjMask.OBJ_ALL_FLAG
        print(f'向 {CONTROLLER_IP} 请求全部急停：双臂、头部、躯干、升降机构', flush=True)
        ret = stop_all(GentoRobot(), CONTROLLER_IP, mask)
        missing = mask & ~ret
        if missing:
            print(f'全部急停未确认：返回掩码=0x{ret:X}，未确认掩码=0x{missing:X}；'
                  '请使用硬件急停！', file=sys.stderr)
            return 1
        print(f'SDK 已确认全部对象的急停请求（掩码=0x{ret:X}）。'
              '请现场确认停止状态；本脚本不解除急停。')
        return 0
    except KeyboardInterrupt:
        print('操作中断，急停结果未确认；请使用硬件急停！', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'急停结果未确认：{exc}；请使用硬件急停！', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
