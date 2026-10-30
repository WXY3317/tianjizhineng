"""Exercise the actual generated migration script against an isolated workspace."""
import ast
from pathlib import Path


def test_deploy_archives_removed_package_and_entrypoints(tmp_path):
    workspace=Path(__file__).resolve().parents[3]
    source=(workspace/'deploy_arm.sh').read_text().split("<<'PY'\n",1)[1].rsplit('\nPY',1)[0]
    tree=ast.parse(source)
    assignments=[node for node in ast.walk(tree) if isinstance(node,ast.Assign)
                 and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ('obsolete','cleanup')]
    environment={'remote_dir':str(tmp_path)}
    for name in ('obsolete','cleanup'):
        statement=next(node for node in assignments if node.targets[0].id==name)
        exec(compile(ast.Module(body=[statement],type_ignores=[]),'<deployment-migration>','exec'),environment)
    ros_workspace=tmp_path/'ros2_ws'
    for relative in ('src/gento_dual_arm/package.xml','install_real/gento_dual_arm/lib/gento_dual_arm/bridge',
                     'build_real/gento_dual_arm/setup.py','install/gento_dual_arm/old-entry',
                     'src/gento_driver/gento_driver/node.py'):
        path=ros_workspace/relative
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(relative)
    exec(environment['cleanup'],{})
    assert not (ros_workspace/'src/gento_dual_arm').exists()
    assert not (ros_workspace/'install_real/gento_dual_arm').exists()
    assert (ros_workspace/'src/gento_driver/gento_driver/node.py').is_file()
    archive=list((ros_workspace/'artifacts').glob('retired-split-*'))
    assert len(archive)==1
    assert (archive[0]/'src/gento_dual_arm/package.xml').is_file()
    assert (archive[0]/'install_real/gento_dual_arm/lib/gento_dual_arm/bridge').is_file()
    exec(environment['cleanup'],{})
    assert len(list((ros_workspace/'artifacts').glob('retired-split-*')))==1


def test_deploy_preflight_refuses_running_robot_without_stopping_it():
    import builtins
    from types import SimpleNamespace
    import pytest
    workspace=Path(__file__).resolve().parents[3]
    source=(workspace/'deploy_arm.sh').read_text().split("<<'PY'\n",1)[1].rsplit('\nPY',1)[0]
    block=next(node.value.value for node in ast.walk(ast.parse(source))
               if isinstance(node,ast.AugAssign) and isinstance(node.value,ast.Constant)
               and 'CHECK_ROBOT_PROCESSES' in str(node.value.value))
    code=block.split('\n',1)[1].rsplit('CHECK_ROBOT_PROCESSES',1)[0]
    class Process:
        name='123'
        def __truediv__(self,name):
            return SimpleNamespace(read_bytes=lambda:b'/usr/bin/python3\0/home/nvidia/ws/install_real/gento_driver/lib/gento_driver/bridge\0')
    def imports(name,*args,**kwargs):
        if name=='pathlib':return SimpleNamespace(Path=lambda path:SimpleNamespace(iterdir=lambda:[Process()]))
        return builtins.__import__(name,*args,**kwargs)
    with pytest.raises(SystemExit,match='Stop robot nodes before deploying'):
        exec(compile(code,'<deployment-preflight>','exec'),{'__builtins__':dict(vars(builtins),__import__=imports)})
