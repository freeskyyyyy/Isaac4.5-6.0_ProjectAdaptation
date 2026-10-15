#!/usr/bin/env python3
"""Migrate complete composed ROS graphs inside the Isaac Sim 6.0 runtime."""
import argparse
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--camera-hz', type=float, default=30.0)
    args = parser.parse_args()
    # SimulationApp forwards unknown CLI args to Kit; a positional USD otherwise
    # opens the legacy graph before extensions and migration are ready.
    sys.argv = [sys.argv[0]]
    # A validated copy is only written after a complete stage has loaded.
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting',
                         'enable_crashreporter': False})
    import omni.usd
    import omni.graph.core as og
    import omni.kit.app
    from pxr import Sdf, Usd, UsdPhysics
    from scene_audit import audit
    from rebuild_graphs import prepare_stage, rebuild

    report = {'status': 'failed', 'changes': [], 'errors': []}
    try:
        print('COURSE_MIGRATION: enabling extensions', flush=True)
        original = audit(args.source)
        if original['missing_assets']:
            report['status'] = 'blocked_missing_assets'
            report['missing_assets'] = original['missing_assets']
            return 2
        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in ('isaacsim.core.nodes', 'isaacsim.sensors.physics.nodes',
                          'isaacsim.robot.wheeled_robots.nodes'):
            if not manager.set_extension_enabled_immediate(extension, True):
                # Some versions return None on success; inspect the actual enabled state.
                if not manager.is_extension_enabled(extension):
                    raise RuntimeError(f'Cannot enable extension: {extension}')
        context = omni.usd.get_context()
        prepared, graphs = prepare_stage(args.source, report)
        print('COURSE_MIGRATION: opening source stage', flush=True)
        if not context.open_stage(str(prepared.resolve())):
            raise RuntimeError('Cannot open source stage')
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        print('COURSE_MIGRATION: stage loaded; checking composition', flush=True)
        # The composed stage includes active branches only; dormant teaching robots
        # must not block migration through their unused external references.
        composition_errors = []
        for error in stage.GetCompositionErrors():
            site = getattr(error, 'rootSite', None)
            owner = stage.GetPrimAtPath(site.path.GetPrimPath()) if site is not None else None
            if owner and not owner.IsActive():
                report.setdefault('inactive_composition_warnings', []).append(str(error))
                continue
            composition_errors.append(str(error))
        if composition_errors:
            raise RuntimeError(f'Active stage composition failed: {composition_errors}')
        # The isolated 6.0 ARM64 check loads geometry before ROS/image nodes.
        # Loading ROS first crashed in the native image-graph stage callback.
        for extension in ('isaacsim.ros2.core', 'isaacsim.ros2.nodes', 'isaacsim.sensors.physx'):
            manager.set_extension_enabled_immediate(extension, True)
            if not manager.is_extension_enabled(extension):
                raise RuntimeError(f'Cannot enable extension: {extension}')
        for _ in range(10):
            app.update()
        print('COURSE_MIGRATION: rebuilding graphs against 6.0 schemas', flush=True)
        rebuild(stage, graphs, og, report)

        keyboard = stage.GetPrimAtPath('/Graphs/kaya_holonomic_controller')
        if keyboard and keyboard.IsActive():
            keyboard.SetActive(False)
            report['changes'].append({'path': str(keyboard.GetPath()),
                'reason': 'Disable keyboard wheel commands competing with the course ROS controller'})
        roots = [p.GetPath() for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]

        def articulation_target(targets):
            if len(targets) != 1:
                raise RuntimeError(f'Expected one articulation target: {targets}')
            target = stage.GetPrimAtPath(targets[0])
            if target and target.HasAPI(UsdPhysics.ArticulationRootAPI):
                return targets
            if len(roots) != 1:
                raise RuntimeError(f'Cannot resolve articulation root for {targets}; roots={roots}')
            report['changes'].append({'old_articulation_target': str(targets[0]),
                                      'new_articulation_target': str(roots[0])})
            return roots

        # Node names are resolved from the loaded 6.0 registry, not guessed aliases.
        registry = og.get_registered_nodes()
        names = [str(x) for x in registry]

        def registered(suffix):
            matches = [name for name in names if name.endswith('.' + suffix)
                       and name.startswith('isaacsim.')]
            preferred = [name for name in matches if not name.startswith('omni.')]
            if len(preferred) != 1:
                raise RuntimeError(f'Ambiguous/missing 6.0 node {suffix}: {matches}')
            return preferred[0]

        for prim in list(stage.Traverse()):
            type_attr = prim.GetAttribute('node:type')
            node_type = str(type_attr.Get() or '') if type_attr else ''
            if node_type.startswith('omni.isaac.'):
                replacement = registered(node_type.rsplit('.', 1)[-1])
                type_attr.Set(replacement)
                report['changes'].append({'path': str(prim.GetPath()), 'node_type': replacement})
            suffix = node_type.rsplit('.', 1)[-1]
            def set_input(name, value, value_type=None):
                attribute = prim.GetAttribute('inputs:' + name)
                if not attribute and value_type is not None:
                    attribute = prim.CreateAttribute('inputs:' + name, value_type)
                if not attribute:
                    raise RuntimeError(f'Missing input {name} on {prim.GetPath()}')
                attribute.Set(value)

            if suffix == 'ROS2Context':
                set_input('domain_id', 44)
                set_input('useDomainIDEnvVar', True)
            if suffix == 'IsaacReadSimulationTime':
                set_input('resetOnStop', True)
            if suffix == 'IsaacCreateRenderProduct':
                # The 6.0 reference camera graph executes RenderProduct every
                # playback tick. Retire the 4.5 one-frame initializer so helper
                # setup can finish after render vars/writers become available.
                ticks = [p for p in prim.GetParent().GetChildren()
                         if p.GetAttribute('node:type').Get() == 'omni.graph.action.OnPlaybackTick']
                if len(ticks) != 1:
                    raise RuntimeError(f'Ambiguous camera tick: {prim.GetPath()}')
                prim.GetAttribute('inputs:execIn').SetConnections(
                    [ticks[0].GetPath().AppendProperty('outputs:tick')])
                report['changes'].append({'render_product': str(prim.GetPath()),
                                           'execution': 'every playback tick (6.0 reference graph)'})
            if suffix == 'IsaacArticulationController':
                relation = prim.GetRelationship('inputs:targetPrim')
                if relation and relation.GetTargets():
                    relation.SetTargets(articulation_target(relation.GetTargets()))
            if suffix in ('ROS2CameraHelper', 'ROS2CameraInfoHelper'):
                kind = 'camera_info' if suffix == 'ROS2CameraInfoHelper' else str(prim.GetAttribute('inputs:type').Get())
                if kind not in ('rgb', 'depth', 'camera_info'):
                    raise RuntimeError(f'Unexpected course camera type {kind}: {prim.GetPath()}')
                set_input('nodeNamespace', '')
                set_input('topicName', '/arm_camera/' + kind)
                set_input('frameId', 'arm_Camera')
                set_input('frameSkipCount', 0)
                set_input('useSystemTime', False)
                set_input('resetSimulationTimeOnStop', True)
            if suffix == 'ROS2SubscribeTwist':
                set_input('nodeNamespace', '')
                set_input('topicName', '/cmd_vel')
            if suffix == 'ROS2PublishOdometry':
                set_input('odomFrameId', 'odom')
                set_input('chassisFrameId', 'base_link')
                for field in ('position', 'orientation'):
                    attribute = prim.GetAttribute('inputs:' + field)
                    connections = [p for p in attribute.GetConnections() if '/ComputeOdometry.' in str(p)]
                    if len(connections) != 1:
                        raise RuntimeError(f'Odometry {field} source is ambiguous: {prim.GetPath()}')
                    attribute.SetConnections(connections)
            if suffix == 'ROS2PublishRawTransformTree':
                set_input('parentFrameId', 'odom')
            if suffix not in ('ROS2PublishTransformTree', 'ROS2PublishJointState'):
                continue
            is_tf = suffix == 'ROS2PublishTransformTree'
            old_name = 'inputs:targetPrims' if is_tf else 'inputs:targetPrim'
            old_target = prim.GetRelationship(old_name)
            targets = old_target.GetTargets() if old_target else []
            if not is_tf and targets:
                is_arm = '/ECO65_B/' in str(targets[0]) or str(prim.GetAttribute('inputs:nodeNamespace').Get()) == 'arm'
                set_input('nodeNamespace', '')
                set_input('topicName', '/arm/joint_states' if is_arm else '/joint_states')
                targets = articulation_target(targets)
            if not targets:
                # Already migrated graphs must have a source connection.
                required = 'inputs:translations' if is_tf else 'inputs:jointPositions'
                if not prim.GetAttribute(required).GetConnections():
                    raise RuntimeError(f'Publisher has no target or data source: {prim.GetPath()}')
                continue
            parent = str(prim.GetParent().GetPath())
            source_name = 'course_source_' + prim.GetName()
            source_type = registered('IsaacComputeTransformTree' if is_tf else 'IsaacReadJointState')
            edits = {og.Controller.Keys.CREATE_NODES: [(source_name, source_type)]}
            og.Controller.edit(og.get_graph_by_path(parent), edits)
            source = stage.GetPrimAtPath(parent + '/' + source_name)
            source.GetRelationship('inputs:targetPrims' if is_tf else 'inputs:prim').SetTargets(targets)
            if is_tf:
                old_parent = prim.GetRelationship('inputs:parentPrim')
                if old_parent and old_parent.GetTargets():
                    source.GetRelationship('inputs:parentPrim').SetTargets(old_parent.GetTargets())
                    old_parent.ClearTargets(True)
                # This assembled robot puts ArticulationRootAPI on the chassis
                # rigid body. In the 6.0 runtime that target emitted only a
                # base_link -> base_link pair instead of expanding sibling
                # links. Enumerate the actual rigid bodies in this assembly;
                # their current poses still come from ComputeTransformTree.
                parents = source.GetRelationship('inputs:parentPrim').GetTargets()
                if len(targets) == 1 and targets == parents and targets == roots:
                    assembly = stage.GetPrimAtPath(roots[0].GetParentPath())
                    bodies = [p.GetPath() for p in Usd.PrimRange(assembly)
                              if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetPath() != roots[0]]
                    if not bodies:
                        raise RuntimeError('No robot link bodies found for transform publication')
                    source.GetRelationship('inputs:targetPrims').SetTargets(bodies)
                    report['changes'].append({'explicit_robot_tf_bodies': [str(p) for p in bodies],
                        'reason': 'Runtime root target emitted only a self transform'})
            exec_attr = prim.GetAttribute('inputs:execIn')
            execution = exec_attr.GetConnections()
            source.GetAttribute('inputs:execIn').SetConnections(execution)
            fields = ['parentFrames', 'childFrames', 'translations', 'orientations'] if is_tf else [
                'jointNames', 'jointPositions', 'jointVelocities', 'jointEfforts',
                'jointDofTypes', 'stageMetersPerUnit', 'sensorTime']
            exec_attr.SetConnections([source.GetPath().AppendProperty('outputs:execOut')])
            for field in fields:
                target = prim.GetAttribute('inputs:' + field)
                origin = source.GetAttribute('outputs:' + field)
                if not target or not origin:
                    raise RuntimeError(f'6.0 schema field missing: {prim.GetPath()} / {field}')
                target.SetConnections([origin.GetPath()])
            old_target.ClearTargets(True)
            report['changes'].append({'publisher': str(prim.GetPath()), 'source': str(source.GetPath())})

        # Do not guess scan rates or silently substitute a different Lidar asset.
        for prim in stage.Traverse():
            if prim.GetTypeName() == 'Camera' and prim.GetName() == 'arm_Camera':
                if args.camera_hz <= 0:
                    raise RuntimeError('Camera frequency must be positive')
                # tickRate is consumed by the 6.0 sensor scheduler only when
                # the Camera carries OmniSensorAPI (see official RtxCamera).
                if not prim.ApplyAPI('OmniSensorAPI'):
                    raise RuntimeError(f'Cannot apply OmniSensorAPI: {prim.GetPath()}')
                prim.CreateAttribute('omni:sensor:tickRate', Sdf.ValueTypeNames.Float).Set(args.camera_hz)
                report['changes'].append({'camera': str(prim.GetPath()), 'tickRate': args.camera_hz,
                                          'schema': 'OmniSensorAPI'})
            legacy_config = prim.GetAttribute('sensorModelConfig')
            if prim.GetTypeName() == 'Camera' and legacy_config and legacy_config.Get():
                raise RuntimeError(f'Legacy RTX Lidar requires JSON-to-USD conversion: {prim.GetPath()}')
            skip = prim.GetAttribute('inputs:frameSkipCount')
            if skip and (skip.Get() or 0) != 0:
                raise RuntimeError(f'Set a sensor tickRate before removing frameSkipCount: {prim.GetPath()}')

        args.output.parent.mkdir(parents=True, exist_ok=True)
        print('COURSE_MIGRATION: settling rebuilt graphs', flush=True)
        for _ in range(10):
            app.update()
        print('COURSE_MIGRATION: exporting migrated stage', flush=True)
        # Preserve portable asset references and author the fresh graphs and
        # overrides in the root layer; the original robot USD stays unchanged.
        migrated = stage.GetRootLayer()
        migrated.customLayerData = {**dict(migrated.customLayerData), 'courseMigration': {
            'status': 'graph_migrated_runtime_acceptance_pending', 'targetIsaacSim': '6.0.0'}}
        if not migrated.Export(str(args.output.resolve())):
            raise RuntimeError('Cannot export migrated stage')
        print('COURSE_MIGRATION: reopening saved stage', flush=True)
        if not context.close_stage():
            raise RuntimeError('Cannot close stage before reopen validation')
        for _ in range(10):
            app.update()
        if not context.open_stage(str(args.output.resolve())):
            raise RuntimeError('Cannot reopen migrated stage')
        for _ in range(10):
            app.update()
        report['status'] = 'graph_migrated_runtime_acceptance_pending'
        report['output'] = str(args.output.resolve())
        return 0
    except Exception as exc:
        report['errors'].append(str(exc))
        return 1
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        app.close(exit_code=0 if report['status'] == 'graph_migrated_runtime_acceptance_pending'
                  else (2 if report['status'] == 'blocked_missing_assets' else 1))


if __name__ == '__main__':
    raise SystemExit(main())
