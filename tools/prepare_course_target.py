#!/usr/bin/env python3
"""Prepare a separate course scene with a real dynamic official YCB marker.

USD-only authoring; this does not certify physics, recognition or grasping.
Existing scenes/assets remain unchanged. Run with /opt/course/usd-python.sh.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--asset',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--x',type=float,default=1.4)
    p.add_argument('--y',type=float,default=.6)
    a=p.parse_args()
    if a.scene.resolve()==a.output.resolve() or a.scene.parent.resolve()!=a.output.parent.resolve():
        p.error('Use a separate output in the input scene directory to preserve all relative references')
    original=hashlib.sha256(a.scene.read_bytes()).hexdigest()
    asset=Usd.Stage.Open(str(a.asset.resolve()))
    if UsdGeom.GetStageMetersPerUnit(asset)!=1. or UsdGeom.GetStageUpAxis(asset)!='Z':
        raise RuntimeError('Marker must use metre units and Z-up')
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy])
    box=cache.ComputeWorldBound(asset.GetDefaultPrim()).ComputeAlignedBox()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    if not Sdf.Layer.FindOrOpen(str(a.scene.resolve())).Export(str(a.output.resolve())):
        raise RuntimeError('Cannot copy scene layer')
    stage=Usd.Stage.Open(str(a.output.resolve()),load=Usd.Stage.LoadNone)
    if stage.GetPrimAtPath('/World/CourseMarker'):
        raise RuntimeError('Input already contains a CourseMarker; do not overwrite an existing object')
    table=stage.GetPrimAtPath('/World/table2/Cube_01')
    top=cache.ComputeWorldBound(table).ComputeAlignedBox()
    if not (top.GetMin()[0]-box.GetMin()[0]<a.x<top.GetMax()[0]-box.GetMax()[0]
            and top.GetMin()[1]-box.GetMin()[1]<a.y<top.GetMax()[1]-box.GetMax()[1]):
        raise RuntimeError('Marker must fit fully inside the table top')
    marker=UsdGeom.Xform.Define(stage,'/World/CourseMarker')
    prim=marker.GetPrim()
    prim.GetReferences().AddReference(os.path.relpath(a.asset.resolve(),a.output.parent.resolve()))
    prim.SetInstanceable(False)
    position=(a.x,a.y,float(top.GetMax()[2]-box.GetMin()[2]+.003))
    UsdGeom.Xformable(prim).ClearXformOpOrder()
    marker.AddTranslateOp().Set(Gf.Vec3d(*position))
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.02)
    material=UsdShade.Material.Define(stage,'/World/Looks/CourseMarkerPhysics')
    physics=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(.8);physics.CreateDynamicFrictionAttr(.6);physics.CreateRestitutionAttr(0.)
    colliders=[]
    for child in Usd.PrimRange(prim):
        if child.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(child).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr('convexHull')
            UsdShade.MaterialBindingAPI.Apply(child).Bind(material,materialPurpose='physics')
            colliders.append(str(child.GetPath()))
    if not colliders:raise RuntimeError('No marker collision meshes found')
    prim.SetCustomDataByKey('courseValidationTarget',True)
    stage.GetRootLayer().Save()
    if hashlib.sha256(a.scene.read_bytes()).hexdigest()!=original:raise RuntimeError('Input scene unexpectedly changed')
    report={'status':'authored_runtime_pending','scope':'official marker rigid-body scene candidate; not runtime acceptance',
        'source':str(a.scene),'source_sha256':original,'output':str(a.output),
        'output_sha256':hashlib.sha256(a.output.read_bytes()).hexdigest(),'asset':str(a.asset),
        'prim':'/World/CourseMarker','authored_position_m':position,'mass_kg':.02,'collision_meshes':colliders,
        'table_top_z_m':float(top.GetMax()[2]),'marker_bbox_min':list(box.GetMin()),'marker_bbox_max':list(box.GetMax()),
        'pending':['Kit texture/appearance check','Drop/settling and reset','Camera recognition and capture-time TF','Physical grasp/lift']}
    a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
