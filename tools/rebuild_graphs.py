"""Rebuild authored course graphs against registered 6.0 schemas before ticking Kit."""
def prepare_stage(source, report):
    from pxr import Usd, UsdPhysics
    stage = Usd.Stage.Open(str(source.resolve()))
    if stage is None:
        raise RuntimeError(f'Cannot read source stage: {source}')
    # This course authored identical /World/PhysicsScene and /PhysicsScene.
    # Runtime diagnostics showed doubled Fabric time and stalled images. Test
    # consolidation only for this known duplicate pair, with matching settings.
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) > 1:
        if {str(p.GetPath()) for p in scenes} != {'/World/PhysicsScene', '/PhysicsScene'}:
            raise RuntimeError('Unexpected multiple PhysicsScenes require explicit ownership review')
        def settings(prim):
            attrs = {}
            for attr in prim.GetAttributes():
                if attr.GetTimeSamples() or attr.GetConnections():
                    raise RuntimeError(f'Dynamic physics settings require review: {attr.GetPath()}')
                if attr.HasAuthoredValueOpinion():
                    attrs[attr.GetName()] = attr.Get()
            return {'attributes': attrs, 'schemas': sorted(prim.GetAppliedSchemas()),
                    'relationships': {r.GetName(): list(r.GetTargets()) for r in prim.GetRelationships()}}
        if any(settings(p) != settings(scenes[0]) for p in scenes[1:]):
            raise RuntimeError('Multiple nonidentical PhysicsScenes require explicit ownership review')
        kept = next((p for p in scenes if str(p.GetPath()) == '/World/PhysicsScene'), scenes[0])
        removed = {p.GetPath() for p in scenes if p != kept}
        for prim in stage.Traverse():
            owner = prim.GetRelationship('physics:simulationOwner')
            if owner and any(p in removed for p in owner.GetTargets()):
                owner.SetTargets([kept.GetPath() if p in removed else p for p in owner.GetTargets()])
        for prim in scenes:
            if prim != kept:
                prim.SetActive(False)
        report['changes'].append({'physics_scene_kept': str(kept.GetPath()),
            'duplicate_physics_scenes_disabled': sorted(str(p) for p in removed),
            'reason': 'Consolidate known identical scene pair; verify Fabric time and camera streams'})
    keyboard = stage.GetPrimAtPath('/Graphs/kaya_holonomic_controller')
    if keyboard:
        keyboard.SetActive(False)
        report['changes'].append({'path': str(keyboard.GetPath()),
            'reason': 'Disable competing keyboard wheel controller'})
    graphs = []
    graph_paths = [prim.GetPath() for prim in stage.Traverse() if prim.GetTypeName() == 'OmniGraph']
    for path in graph_paths:
        prim = stage.GetPrimAtPath(path)
        nodes = []
        for child in prim.GetChildren():
            node_type = child.GetAttribute('node:type')
            if not node_type:
                continue
            attributes = []
            for attribute in child.GetAttributes():
                name = attribute.GetName()
                if not name.startswith(('inputs:', 'outputs:')):
                    continue
                connections = []
                for connection in attribute.GetConnections():
                    owner = stage.GetPrimAtPath(connection.GetPrimPath())
                    if owner and owner.IsActive():
                        connections.append(connection)
                    else:
                        report.setdefault('inactive_connections_removed', []).append({
                            'source': str(connection), 'target': str(attribute.GetPath())})
                value = attribute.Get() if attribute.HasAuthoredValueOpinion() else None
                if connections or (value is not None and (name.startswith('inputs:') or
                        'Constant' in str(node_type.Get()))):
                    attributes.append((name, value, connections))
            nodes.append({'name': child.GetName(), 'type': str(node_type.Get()),
                'attributes': attributes,
                'relationships': [(r.GetName(), list(r.GetTargets())) for r in child.GetRelationships()]})
        graphs.append({'old': prim.GetPath(), 'nodes': nodes,
            'new': '/CourseGraphs/g' + str(len(graphs)).zfill(2) + '_' + prim.GetName()})
        # Keep original graph definitions dormant in referenced assets. The
        # loading copy contains only geometry and the dormant graph definitions.
    for path in graph_paths:
        stage.GetPrimAtPath(path).SetActive(False)
    prepared = source.with_name('.course-load-prepared.usda')
    if not stage.GetRootLayer().Export(str(prepared)):
        raise RuntimeError('Cannot save graph-disabled loading copy')
    report['original_graphs_disabled'] = [str(g['old']) for g in graphs]
    return prepared, graphs


def rebuild(stage, graphs, og, report):
    from pxr import Sdf
    registered = set(og.get_registered_nodes())

    def canonical(name):
        if name in registered:
            return name
        suffix = name.rsplit('.', 1)[-1]
        matches = [n for n in registered if n.startswith('isaacsim.') and n.endswith('.' + suffix)]
        if len(matches) != 1:
            raise RuntimeError(f'Unregistered course node {name}; candidates={matches}')
        return matches[0]

    def translated(path):
        for graph in graphs:
            if path.HasPrefix(graph['old']):
                return path.ReplacePrefix(graph['old'], Sdf.Path(graph['new']))
        return path

    # Construct all graphs and nodes before restoring connections across graphs.
    for graph in graphs:
        og.Controller.edit({'graph_path': graph['new'], 'evaluator_name': 'execution'}, {
            og.Controller.Keys.CREATE_NODES: [(n['name'], canonical(n['type'])) for n in graph['nodes']]})
    for graph in graphs:
        for node in graph['nodes']:
            prim = stage.GetPrimAtPath(graph['new'] + '/' + node['name'])
            for name, value, connections in node['attributes']:
                attribute = prim.GetAttribute(name)
                if not attribute:
                    report.setdefault('obsolete_ports_removed', []).append(str(prim.GetPath()) + '.' + name)
                    continue
                if connections:
                    # The original scene connected both a conditional and the
                    # odometry source to scalar ports; retain the real odometry.
                    if node['type'].endswith('.ROS2PublishOdometry') and name in ('inputs:position', 'inputs:orientation'):
                        connections = [p for p in connections if '/ComputeOdometry.' in str(p)]
                    resolved = [translated(p) for p in connections]
                    for connection in resolved:
                        if not stage.GetAttributeAtPath(connection):
                            raise RuntimeError(f'Connection source missing: {connection} -> {prim.GetPath()}.{name}')
                    attribute.SetConnections(resolved)
                elif value is not None and name not in ('inputs:execIn', 'outputs:execOut'):
                    try:
                        attribute.Set(value)
                    except Exception as exc:
                        raise RuntimeError(f'Cannot restore {prim.GetPath()}.{name}: {exc}') from exc
            for name, targets in node['relationships']:
                relation = prim.GetRelationship(name)
                if relation:
                    relation.SetTargets([translated(p) for p in targets])
        report['changes'].append({'graph_rebuilt': graph['new'], 'source': str(graph['old']),
                                  'node_count': len(graph['nodes'])})
