#!/usr/bin/env python3
"""Open the migrated course scene with Isaac Sim GUI; ROS runs separately."""
import argparse
import json
import sys
from pathlib import Path


def capture_diagnostics(path):
    """A diagnostic failure must never close the simulator or alter the stage."""
    import carb
    import omni.graph.core as og
    import omni.timeline
    import omni.usd
    from isaacsim.ros2.nodes.ogn.OgnROS2CameraHelperDatabase import OgnROS2CameraHelperDatabase

    stage = omni.usd.get_context().get_stage()
    result = {'simulation_time': omni.timeline.get_timeline_interface().get_current_time(),
              'playing': omni.timeline.get_timeline_interface().is_playing(),
              'srtx': carb.settings.get_settings().get('/exts/omni.replicator.srtx/enabled'),
              'nodes': [], 'cameras': []}
    result['settings'] = {name: carb.settings.get_settings().get(name) for name in (
        '/omni/replicator/captureOnPlay', '/app/player/playSimulations',
        '/app/settings/fabricDefaultStageFrameHistoryCount', '/rtx/hydra/supportMultiTickRate',
        '/app/renderer/skipWhileMinimized')}
    import usdrt
    fabric = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    external_time = fabric.GetPrimAtPath('/ExternalSimulationTime')
    result['fabric_simulation_time'] = (external_time.GetAttribute('omni:time').Get()
        if external_time and external_time.HasAttribute('omni:time') else None)
    for prim in stage.Traverse():
        if prim.GetTypeName() == 'Camera':
            result['cameras'].append({'path': str(prim.GetPath()),
                'schemas': prim.GetAppliedSchemas(),
                'tick_rate': prim.GetAttribute('omni:sensor:tickRate').Get()})
        attr = prim.GetAttribute('node:type')
        kind = str(attr.Get() or '') if attr else ''
        if not any(s in kind for s in ('CameraHelper', 'SimulationGate', 'CreateRenderProduct',
                                      'PublishImage', 'Dispatch', 'ReferenceTime', 'SyncGate')):
            continue
        item = {'path': str(prim.GetPath()), 'type': kind, 'ports': {}}
        for name in ('inputs:enabled', 'inputs:step', 'outputs:renderProductPath',
                     'inputs:renderProductPath', 'inputs:type', 'inputs:width', 'inputs:height',
                     'outputs:referenceTimeNumerator', 'outputs:referenceTimeDenominator',
                     'inputs:simTimesToWrite', 'inputs:prevSimTimesToWrite'):
            if prim.GetAttribute(name):
                item['ports'][name] = str(og.Controller.get(str(prim.GetPath()) + '.' + name))
        if kind.endswith('.ROS2CameraHelper'):
            node = og.Controller.node(str(prim.GetPath()))
            state = OgnROS2CameraHelperDatabase.per_instance_internal_state(node)
            if state is None:
                item['writer_state'] = {'initialized': False}
            else:
                item['writer_state'] = {'initialized': state.initialized,
                    'writers': len(state._writers), 'pending_requests': len(state._requests),
                    'render_var': state.rv, 'publish_step_size': state.publishStepSize}
        result['nodes'].append(item)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, default=str) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--drive-audit', type=Path,
                        help='Evaluate installed drive solver on actual USD geometry without motion')
    parser.add_argument('--configure-drive-copy', type=Path,
                        help='Configure chassis gains and export a separate candidate scene before playing')
    parser.add_argument('--diagnostics', type=Path,
                        help='Write periodic camera/render graph diagnostics while playing')
    args = parser.parse_args()
    # Open the stage explicitly after required extensions have loaded.
    sys.argv = [sys.argv[0]]
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
        for extension in ('isaacsim.ros2.core', 'isaacsim.ros2.nodes', 'isaacsim.ros2.ui',
                          'isaacsim.robot.wheeled_robots.nodes', 'isaacsim.sensors.physx'):
            manager.set_extension_enabled_immediate(extension, True)
            if not manager.is_extension_enabled(extension):
                raise RuntimeError(f'ROS extension did not enable: {extension}')
        # Let native image-graph startup callbacks settle before replacing the
        # empty stage; the migration reopen check uses the same ordering.
        for _ in range(10):
            app.update()
        if not omni.usd.get_context().open_stage(str(args.scene.resolve())):
            raise RuntimeError('Cannot open migrated scene')
        for _ in range(10):
            app.update()
        if args.drive_audit:
            from drive_audit import audit_drive
            audit_drive(args.drive_audit)
        if args.configure_drive_copy:
            if args.configure_drive_copy.resolve() == args.scene.resolve():
                raise RuntimeError('Drive diagnostic must preserve its input scene')
            from drive_audit import configure_drive
            configuration = configure_drive(omni.usd.get_context().get_stage())
            args.configure_drive_copy.parent.mkdir(parents=True, exist_ok=True)
            omni.usd.get_context().get_stage().GetRootLayer().Export(str(args.configure_drive_copy))
            args.configure_drive_copy.with_suffix('.drive.json').write_text(json.dumps(configuration, indent=2)+'\n')
            print('Drive candidate written: '+str(args.configure_drive_copy), flush=True)
        # Match the working NVIDIA periodic-camera reference initialization.
        # PhysX/Lidar use CPU; RTX rendering and YOLOE still use the GPU.
        from isaacsim.core.simulation_manager import SimulationManager
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device='cpu')
        # Use NVIDIA's simulation_interfaces services for automated play/pause/
        # reset validation rather than injecting GUI input or arbitrary scripts.
        extension = 'isaacsim.ros2.sim_control'
        manager.set_extension_enabled_immediate(extension, True)
        if not manager.is_extension_enabled(extension):
            raise RuntimeError(f'Simulation control did not enable: {extension}')
        print('课程场景已打开；按 Play 启动仿真，在另一终端执行 bash tools/course.sh ros。', flush=True)
        updates = 0
        while app.is_running():
            app.update()
            updates += 1
            if (args.diagnostics and updates % 120 == 0
                    and omni.timeline.get_timeline_interface().is_playing()):
                try:
                    capture_diagnostics(args.diagnostics)
                except Exception as exc:
                    print(f'Camera diagnostic snapshot failed: {exc}', flush=True)
    except Exception:
        # Fast Kit shutdown can terminate Python before its normal uncaught
        # exception handler runs. Flush the real failure before closing Kit.
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        # This SDK's fast shutdown exits immediately; preserve failure status.
        error_type = sys.exc_info()[0]
        app.close(exit_code=1 if error_type not in (None, KeyboardInterrupt) else 0)


if __name__ == '__main__':
    main()
