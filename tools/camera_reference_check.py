#!/usr/bin/env python3
"""Isolate ROS camera helpers using NVIDIA's 6.0 graph and setup APIs.

Reference: SDK standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
and isaacsim.ros2.nodes/tests/test_camera.py. No course assets or motion commands.
"""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--scene', type=Path, help='Load course geometry and disable its graphs in memory only')
    parser.add_argument('--camera-prim', help='Use an existing camera instead of creating the reference camera')
    parser.add_argument('--course-camera-graph', action='store_true',
                        help='Isolate the saved course camera graph instead of creating a reference graph')
    parser.add_argument('--keep-course-graphs', action='store_true',
                        help='With --course-camera-graph, retain all course graphs for comparison')
    parser.add_argument('--sim-control', action='store_true',
                        help='Enable the same official simulation-control extension as the normal runner')
    parser.add_argument('--observer-delay', type=float, default=0,
                        help='Run after Play before creating image subscribers, to test late observers')
    parser.add_argument('--play-delay', type=float, default=0,
                        help='Update the paused application before Play, as with the interactive runner')
    parser.add_argument('--service-play', action='store_true',
                        help='Play through the official ROS service instead of direct timeline.play')
    parser.add_argument('--external-observer', action='store_true',
                        help='Do not subscribe/spin in Kit; verify streams with a separate ROS process')
    args = parser.parse_args()
    if args.course_camera_graph and not args.scene:
        parser.error('--course-camera-graph requires --scene')
    if args.keep_course_graphs and not args.course_camera_graph:
        parser.error('--keep-course-graphs requires --course-camera-graph')
    if args.service_play and not args.sim_control:
        parser.error('--service-play requires --sim-control')
    sys.argv = [sys.argv[0]]
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': False})
    report = {'status': 'failed', 'scope': 'minimal SDK reference, not course acceptance', 'errors': []}
    node = None
    status = 1
    try:
        import omni.kit.app
        import omni.usd
        import omni.timeline
        import omni.graph.core as og
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import UsdGeom, UsdLux, UsdPhysics, Gf
        from usdrt import Sdf

        manager = omni.kit.app.get_app().get_extension_manager()
        for ext in ('isaacsim.ros2.core', 'isaacsim.ros2.nodes'):
            manager.set_extension_enabled_immediate(ext, True)
        # Let image-graph startup/stage callbacks settle before replacing the
        # empty stage, as in the validated migration and normal course runner.
        for _ in range(10):
            app.update()
        if args.scene:
            if not omni.usd.get_context().open_stage(str(args.scene.resolve())):
                raise RuntimeError('Cannot open comparison scene')
            for _ in range(10):
                app.update()
        stage = omni.usd.get_context().get_stage()
        if args.scene and not args.keep_course_graphs:
            for path in ('/CourseGraphs', '/Graphs'):
                prim = stage.GetPrimAtPath(path)
                if prim:
                    if args.course_camera_graph and path == '/CourseGraphs':
                        for child in prim.GetChildren():
                            child.SetActive(child.GetName() == 'g01_ROS_Camera')
                    else:
                        prim.SetActive(False)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(500)
        UsdGeom.Cube.Define(stage, '/World/Cube').CreateSizeAttr(1.0)
        camera_path = args.camera_prim or '/World/ReferenceCamera'
        if args.camera_prim:
            camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
            if not camera:
                raise RuntimeError(f'Camera not found: {camera_path}')
        else:
            camera = UsdGeom.Camera.Define(stage, camera_path)
            camera.GetPrim().ApplyAPI('OmniSensorAPI')
            camera.GetPrim().GetAttribute('omni:sensor:tickRate').Set(30.0)
            UsdGeom.XformCommonAPI(camera).SetTranslate(Gf.Vec3d(0, 0, 3))
        report['scene'] = str(args.scene) if args.scene else None
        report['camera_prim'] = camera_path
        keys = og.Controller.Keys
        create = [('Tick', 'omni.graph.action.OnPlaybackTick'),
                  ('Product', 'isaacsim.core.nodes.IsaacCreateRenderProduct')]
        connect = [('Tick.outputs:tick', 'Product.inputs:execIn')]
        values = [('Product.inputs:cameraPrim', [Sdf.Path(camera_path)]),
                  ('Product.inputs:width', 640), ('Product.inputs:height', 480)]
        for name, kind in [('RGB', 'rgb'), ('Depth', 'depth')]:
            create.append((name, 'isaacsim.ros2.bridge.ROS2CameraHelper'))
            connect += [('Product.outputs:execOut', f'{name}.inputs:execIn'),
                        ('Product.outputs:renderProductPath', f'{name}.inputs:renderProductPath')]
            values += [(f'{name}.inputs:type', kind), (f'{name}.inputs:topicName', f'/diagnostics/camera/{kind}'),
                       (f'{name}.inputs:frameId', 'reference_camera_optical'),
                       (f'{name}.inputs:resetSimulationTimeOnStop', True)]
        if not args.course_camera_graph:
            og.Controller.edit({'graph_path': '/ReferenceGraph', 'evaluator_name': 'execution'},
                               {keys.CREATE_NODES: create, keys.CONNECT: connect, keys.SET_VALUES: values})
        report['camera_graph'] = ('all saved course graphs' if args.keep_course_graphs else
            'saved course graph only' if args.course_camera_graph else 'new reference graph')
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device='cpu')
        if args.sim_control:
            manager.set_extension_enabled_immediate('isaacsim.ros2.sim_control', True)
        report['sim_control'] = args.sim_control
        if not rclpy.ok():
            rclpy.init(args=[])
        node = rclpy.create_node('reference_camera_observer')
        arrivals = {'rgb': [], 'depth': []}
        stamps = {'rgb': [], 'depth': []}
        subscriptions = []

        def receive(msg, kind):
            arrivals[kind].append(time.monotonic())
            stamps[kind].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        play_at = time.monotonic() + args.play_delay
        while time.monotonic() < play_at:
            app.update()
        report['play_delay'] = args.play_delay
        if args.service_play:
            from simulation_interfaces.srv import SetSimulationState
            client = node.create_client(SetSimulationState, '/set_simulation_state')
            deadline = time.monotonic() + 15
            while not client.service_is_ready():
                if time.monotonic() >= deadline:
                    raise RuntimeError('Official play service not available')
                app.update()
                rclpy.spin_once(node, timeout_sec=0)
            request = SetSimulationState.Request()
            request.state.state = 1
            future = client.call_async(request)
            while not future.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError('Official play service timed out')
                app.update()
                rclpy.spin_once(node, timeout_sec=0)
            if future.result().result.result != 1:
                raise RuntimeError(f'Official play failed: {future.result()}')
        else:
            omni.timeline.get_timeline_interface().play()
        report['play_method'] = 'ROS service' if args.service_play else 'direct timeline'
        observer_start = time.monotonic() + args.observer_delay
        while time.monotonic() < observer_start:
            app.update()
        report['observer_delay'] = args.observer_delay
        for kind in arrivals:
            if args.external_observer:
                continue
            topic = f'/arm_camera/{kind}' if args.course_camera_graph else f'/diagnostics/camera/{kind}'
            subscriptions.append(node.create_subscription(Image, topic,
                lambda msg, kind=kind: receive(msg, kind), qos_profile_sensor_data))
        started = time.monotonic()
        while time.monotonic() - started < args.duration:
            app.update()
            if not args.external_observer:
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0)
        report['streams'] = {}
        for kind, times in arrivals.items():
            if args.external_observer:
                continue
            unique = sorted(set(stamps[kind]))
            span = unique[-1] - unique[0] if len(unique) > 1 else 0
            wall_span = times[-1] - times[0] if len(times) > 1 else 0
            max_gap = max((b - a for a, b in zip(times, times[1:])), default=None)
            report['streams'][kind] = {'messages': len(times), 'unique_stamps': len(unique),
                'sim_span': span, 'wall_span': wall_span, 'max_wall_gap': max_gap,
                'hz_sim': (len(unique) - 1) / span if span > 0 else None}
            if wall_span < args.duration * 0.75 or max_gap is None or max_gap > 1:
                report['errors'].append(f'Reference {kind} stream is not sustained')
        status = 0 if not report['errors'] else 1
        report['status'] = 'passed' if status == 0 else 'failed'
        if args.external_observer and status == 0:
            report['status'] = 'external_observation_required'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        if node is not None:
            node.destroy_node()
        app.close(exit_code=status)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
