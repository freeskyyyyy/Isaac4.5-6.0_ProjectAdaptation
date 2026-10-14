#!/usr/bin/env python3
"""Audit authored USD layers without a GPU; never hide missing references."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
from urllib.parse import urlparse

from pxr import Sdf, Usd, UsdUtils


def audit(source):
    source = Path(source).resolve()
    sdk_version = Path('/isaac-sim/VERSION')
    result = {'source': str(source), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'audit_usd_version': list(Usd.GetVersion()),
              'audit_sdk_version': sdk_version.read_text().strip() if sdk_version.is_file() else None,
              'target_isaac_sim': '6.0.0', 'layers': [], 'assets': [], 'nodes': [],
              'sensor_candidates': [], 'inherited_graph_overrides': [], 'errors': []}
    visited = set()

    def visit_file(path):
        path = Path(path).resolve()
        if path in visited:
            return
        visited.add(path)
        layer = Sdf.Layer.FindOrOpen(str(path))
        if layer is None:
            result['errors'].append(f'Cannot open USD layer: {path}')
            return
        result['layers'].append(str(path))
        dependencies = {value for value in layer.GetExternalReferences() if value}
        active_dependencies, inactive_dependencies = set(), set()

        def is_active(prim_path):
            while prim_path != Sdf.Path.absoluteRootPath:
                ancestor = layer.GetPrimAtPath(prim_path)
                if ancestor is not None and ancestor.HasInfo('active') and not ancestor.active:
                    return False
                prim_path = prim_path.GetParentPath()
            return True

        def record(value, owner):
            if value:
                dependencies.add(value)
                (active_dependencies if is_active(owner) else inactive_dependencies).add(value)

        def visit_spec(spec_path):
            spec = layer.GetObjectAtPath(spec_path)
            if isinstance(spec, Sdf.AttributeSpec):
                value = spec.default
                if isinstance(value, Sdf.AssetPath) and value.path:
                    record(value.path, spec.path.GetPrimPath())
                elif spec.typeName == Sdf.ValueTypeNames.AssetArray and value is not None:
                    for asset in value:
                        record(asset.path, spec.path.GetPrimPath())
            if not isinstance(spec, Sdf.PrimSpec):
                return
            for editor in (spec.referenceList, spec.payloadList):
                for item in editor.GetAppliedItems():
                    record(item.assetPath, spec.path)
            if not is_active(spec.path):
                return
            node = spec.attributes.get('node:type')
            if node is not None and node.default:
                values = {a.name: str(a.default) for a in spec.attributes.values()
                          if a.name in ('inputs:topicName', 'inputs:frameId', 'inputs:frameSkipCount',
                                        'inputs:fullScan', 'inputs:robotPath', 'inputs:primPath')}
                result['nodes'].append({'layer': str(path), 'path': str(spec.path),
                                        'type': str(node.default), 'inputs': values})
            elif spec.specifier == Sdf.SpecifierOver and '/Graph/' in str(spec.path):
                if spec.attributes or spec.relationships:
                    result['inherited_graph_overrides'].append(str(spec.path))
            if ('lidar' in str(spec.path).lower() or spec.typeName in ('Camera', 'OmniLidar') or
                    any('sensor' in a.name.lower() or 'lidar' in a.name.lower()
                        for a in spec.attributes.values())):
                result['sensor_candidates'].append({'path': str(spec.path), 'type': spec.typeName,
                                                     'layer': str(path)})

        layer.Traverse(Sdf.Path.absoluteRootPath, visit_spec)
        for dep in sorted(dependencies):
            if dep in inactive_dependencies and dep not in active_dependencies:
                result['assets'].append({'layer': str(path), 'authored': dep,
                                        'status': 'inactive_not_required', 'resolved': dep})
                continue
            scheme = urlparse(dep).scheme
            # Kit resolves bundled MDL modules through its search path, not the USD directory.
            if not scheme and Path(dep).suffix == '.mdl' and '/' not in dep:
                result['assets'].append({'layer': str(path), 'authored': dep,
                                        'status': 'resolver_required', 'resolved': dep})
                continue
            if scheme in ('http', 'https', 'omniverse'):
                result['assets'].append({'layer': str(path), 'authored': dep,
                                        'status': 'remote_unverified', 'resolved': dep})
                continue
            if scheme:
                result['assets'].append({'layer': str(path), 'authored': dep,
                                        'status': 'resolver_required', 'resolved': dep})
                continue
            resolved = Path(dep) if Path(dep).is_absolute() else path.parent / dep
            resolved = resolved.resolve()
            exists = resolved.is_file()
            result['assets'].append({'layer': str(path), 'authored': dep, 'resolved': str(resolved),
                                    'status': 'present' if exists else 'missing'})
            if exists and resolved.suffix.lower() in ('.usd', '.usda', '.usdc'):
                visit_file(resolved)

    visit_file(source)
    result['missing_assets'] = [x for x in result['assets'] if x['status'] == 'missing']
    result['node_type_counts'] = dict(Counter(x['type'] for x in result['nodes']))
    result['status'] = 'blocked_missing_assets' if result['missing_assets'] else (
        'failed' if result['errors'] else 'static_audit_passed_runtime_pending')
    return result


def prepare_draft(source, output, asset_root, report, solid_markings=False):
    """Portable draft only; missing inherited graphs cannot be migrated here."""
    source, output, asset_root = map(lambda x: Path(x).resolve(), (source, output, asset_root))
    output.parent.mkdir(parents=True, exist_ok=True)
    original = Sdf.Layer.FindOrOpen(str(source))
    layer = Sdf.Layer.CreateAnonymous('course-draft.usda')
    layer.TransferContent(original)

    def remap(value):
        if not value or urlparse(value).scheme:
            return value
        if Path(value).suffix == '.mdl' and '/' not in value:
            return value
        marker = '/isaacsimassets-dev/assets/'
        if marker in value:
            target = asset_root / value.split(marker, 1)[1]
        else:
            target = Path(value) if Path(value).is_absolute() else source.parent / value
        return os.path.relpath(target, output.parent)

    UsdUtils.ModifyAssetPaths(layer, remap)
    if solid_markings:
        # This input scene's absent JPEG belongs only to the dashed-line material.
        shader = layer.GetPrimAtPath('/World/Looks/xuxian/Shader')
        if shader and shader.attributes.get('inputs:diffuse_texture'):
            shader.attributes['inputs:diffuse_texture'].default = Sdf.AssetPath('')
            if not shader.attributes.get('inputs:diffuse_color_constant'):
                Sdf.AttributeSpec(shader, 'inputs:diffuse_color_constant', Sdf.ValueTypeNames.Color3f)
            shader.attributes['inputs:diffuse_color_constant'].default = (0.9, 0.9, 0.9)
            report['cosmetic_fallback'] = 'Missing dashed-line JPEG replaced by a light solid material; geometry preserved'
    metadata = dict(layer.customLayerData)
    metadata['courseMigration'] = {'status': 'draft_runtime_pending',
                                  'targetIsaacSim': '6.0.0', 'sourceSHA256': report['source_sha256'],
                                  'missingAssetCount': len(report['missing_assets'])}
    layer.customLayerData = metadata
    if not layer.Export(str(output)):
        raise RuntimeError(f'Cannot save draft: {output}')
    report['draft'] = str(output)
    report['draft_is_validated_scene'] = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--draft', type=Path)
    parser.add_argument('--asset-root', type=Path, default=Path('assets'))
    parser.add_argument('--solid-markings', action='store_true',
                        help='Use a solid color for this scene\'s absent dashed-line texture')
    args = parser.parse_args()
    report = audit(args.source)
    if args.draft:
        prepare_draft(args.source, args.draft, args.asset_root, report, args.solid_markings)
        effective = audit(args.draft)
        effective['original_source'] = {'source': report['source'], 'source_sha256': report['source_sha256'],
                                        'missing_assets_before_relink': report['missing_assets']}
        effective['draft'] = report['draft']
        effective['draft_is_validated_scene'] = False
        if 'cosmetic_fallback' in report:
            effective['cosmetic_fallback'] = report['cosmetic_fallback']
        report = effective
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'missing_assets': report['missing_assets'],
                      'node_type_counts': report['node_type_counts']}, ensure_ascii=False, indent=2))
    raise SystemExit(2 if report['missing_assets'] else (1 if report['errors'] else 0))


if __name__ == '__main__':
    main()
