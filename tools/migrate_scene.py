#!/usr/bin/env python3
"""Migrate complete composed ROS graphs inside the Isaac Sim 6.0 runtime."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    # A validated copy is only written after a complete stage has loaded.
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    import omni.usd
    import omni.graph.core as og
    import omni.kit.app
    from pxr import Sdf, UsdUtils
    from scene_audit import audit

    report = {'status': 'failed', 'changes': [], 'errors': []}
    try:
        original = audit(args.source)
        if original['missing_assets']:
            report['status'] = 'blocked_missing_assets'
            report['missing_assets'] = original['missing_assets']
            return 2
        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in ('isaacsim.core.nodes', 'isaacsim.sensors.physics.nodes',
                          'isaacsim.ros2.core', 'isaacsim.ros2.nodes'):
            if not manager.set_extension_enabled_immediate(extension, True):
                # Some versions return None on success; inspect the actual enabled state.
                if not manager.is_extension_enabled(extension):
                    raise RuntimeError(f'Cannot enable extension: {extension}')
        context = omni.usd.get_context()
        if not context.open_stage(str(args.source.resolve())):
            raise RuntimeError('Cannot open source stage')
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        # Runtime dependency resolution also validates remote USD composition.
        _, _, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(args.source.resolve())))
        if unresolved:
            raise RuntimeError(f'Unresolved composed assets: {list(unresolved)}')

        # Node names are resolved from the loaded 6.0 registry, not guessed aliases.
        registry = og.get_all_node_types()
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
            if suffix not in ('ROS2PublishTransformTree', 'ROS2PublishJointState'):
                continue
            is_tf = suffix == 'ROS2PublishTransformTree'
            old_name = 'inputs:targetPrims' if is_tf else 'inputs:targetPrim'
            old_target = prim.GetRelationship(old_name)
            targets = old_target.GetTargets() if old_target else []
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
            og.Controller.edit({'graph_path': parent}, edits)
            source = stage.GetPrimAtPath(parent + '/' + source_name)
            source.GetRelationship('inputs:targetPrims' if is_tf else 'inputs:prim').SetTargets(targets)
            if is_tf:
                old_parent = prim.GetRelationship('inputs:parentPrim')
                if old_parent and old_parent.GetTargets():
                    source.GetRelationship('inputs:parentPrim').SetTargets(old_parent.GetTargets())
                    old_parent.ClearTargets(True)
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
            legacy_config = prim.GetAttribute('sensorModelConfig')
            if prim.GetTypeName() == 'Camera' and legacy_config and legacy_config.Get():
                raise RuntimeError(f'Legacy RTX Lidar requires JSON-to-USD conversion: {prim.GetPath()}')
            skip = prim.GetAttribute('inputs:frameSkipCount')
            if skip and (skip.Get() or 0) != 0:
                raise RuntimeError(f'Set a sensor tickRate before removing frameSkipCount: {prim.GetPath()}')

        args.output.parent.mkdir(parents=True, exist_ok=True)
        # A flattened copy preserves edits to nodes originally inherited from referenced robot assets.
        migrated = stage.Flatten()
        migrated.customLayerData = {**dict(migrated.customLayerData), 'courseMigration': {
            'status': 'graph_migrated_runtime_acceptance_pending', 'targetIsaacSim': '6.0.0'}}
        if not migrated.Export(str(args.output.resolve())):
            raise RuntimeError('Cannot export migrated stage')
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
        app.close()


if __name__ == '__main__':
    raise SystemExit(main())
