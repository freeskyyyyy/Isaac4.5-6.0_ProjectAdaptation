#!/usr/bin/env python3
"""Open the migrated course scene with Isaac Sim GUI; ROS runs separately."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    if not args.scene.is_file():
        raise SystemExit(f'迁移后的场景不存在: {args.scene}。先执行 audit 并补齐资产，再执行 migrate。')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless})
    try:
        import omni.kit.app
        import omni.usd
        import omni.timeline
        from scene_audit import audit
        result = audit(args.scene)
        if result['missing_assets']:
            raise RuntimeError('场景缺失资产: ' + json.dumps(result['missing_assets'], ensure_ascii=False))
        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in ('isaacsim.ros2.core', 'isaacsim.ros2.nodes', 'isaacsim.ros2.ui'):
            manager.set_extension_enabled_immediate(extension, True)
            if not manager.is_extension_enabled(extension):
                raise RuntimeError(f'ROS extension did not enable: {extension}')
        if not omni.usd.get_context().open_stage(str(args.scene.resolve())):
            raise RuntimeError('Cannot open migrated scene')
        print('课程场景已打开；按 Play 启动仿真，在另一终端执行 bash tools/course.sh ros。', flush=True)
        while app.is_running():
            app.update()
    finally:
        app.close()


if __name__ == '__main__':
    main()
