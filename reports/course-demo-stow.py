"""Original course observation posture through MoveIt, with real table boxes."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject,PlanningScene
from moveit_msgs.srv import GetPositionFK,ApplyPlanningScene
from simulation_interfaces.srv import GetEntityState
from eco65b_grasp_interfaces.action import GraspToPoint
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');a=p.parse_args()
rclpy.init();n=rclpy.create_node('course_original_observation',parameter_overrides=[Parameter('use_sim_time',value=True)])
j=[];cmd=[];subs=[n.create_subscription(JointState,'/arm/joint_state_deal',j.append,qos_profile_sensor_data),n.create_subscription(JointState,'/hand_command',cmd.append,10)]
fk=n.create_client(GetPositionFK,'/compute_fk');apply=n.create_client(ApplyPlanningScene,'/apply_planning_scene');getter=n.create_client(GetEntityState,'/get_entity_state');client=ActionClient(n,GraspToPoint,'/grasp_to_point');active=None
r={'status':'failed','scope':'original observation joints via MoveIt with six table collision boxes','execute':a.execute,'errors':[],'observation_joints_rad':[0.,0.,0.,0.,0.,0.]}
def wait(f,timeout=60):
 end=time.monotonic()+timeout
 while not f.done():
  if time.monotonic()>end:raise RuntimeError('Observation service/action wall timeout')
  rclpy.spin_once(n,timeout_sec=.01)
 return f.result()
def rz(yaw):return Rotation.from_euler('z',yaw).as_matrix()
try:
 assert all(c.wait_for_service(timeout_sec=10)for c in [fk,apply,getter])and client.wait_for_server(timeout_sec=10)
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 assert j and len(j[-1].position)==6
 q=wait(getter.call_async(GetEntityState.Request(entity='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link')));assert q.result.result==1
 pose=q.state.pose;v=q.state.twist;assert math.hypot(v.linear.x,v.linear.y)<.02 and abs(v.angular.z)<.03
 rotation=Rotation.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]).as_matrix();base=np.array([pose.position.x,pose.position.y,pose.position.z])
 geometry=json.loads(Path('/workspace/course/bobac_robotac_src/src/bobac3_navigation/maps/world_scene_geometry.json').read_text());trans=geometry['robot_transforms'];actual_arm=wait(getter.call_async(GetEntityState.Request(entity='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/ECO65_B/base_link_arm')));assert actual_arm.result.result==1;ap=actual_arm.state.pose;mount=np.array([ap.position.x,ap.position.y,ap.position.z]);aq=ap.orientation;arm_rotation=Rotation.from_quat([aq.x,aq.y,aq.z,aq.w]).as_matrix()
 scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True
 for index,entry in enumerate(x for x in geometry['collisions']if x['path'].startswith(('/World/table1/','/World/table2/'))):
  xyz=np.asarray(entry['world_points_m']);low=xyz.min(axis=0);high=xyz.max(axis=0);o=CollisionObject(id='course_demo_table_'+str(index),operation=CollisionObject.ADD);o.header.frame_id='base_link_arm';o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=(high-low).tolist())];p=Pose();p.position.x,p.position.y,p.position.z=(arm_rotation.T@((high+low)/2-mount)).tolist();quat=Rotation.from_matrix(arm_rotation.T).as_quat();p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat.tolist();o.primitive_poses=[p];scene.world.collision_objects.append(o)
 assert len(scene.world.collision_objects)==6 and wait(apply.call_async(ApplyPlanningScene.Request(scene=scene))).success
 r['table_collision_ids']=[x.id for x in scene.world.collision_objects];r['base_world_xyz']=base.tolist()
 request=GetPositionFK.Request();request.header.frame_id='base_link_arm';request.fk_link_names=['link_6'];request.robot_state.joint_state.name=['joint_'+str(i)for i in range(1,7)];request.robot_state.joint_state.position=r['observation_joints_rad'];response=wait(fk.call_async(request));assert response.error_code.val==1
 wrist=response.pose_stamped[0].pose;quat=wrist.orientation;rot=Rotation.from_quat([quat.x,quat.y,quat.z,quat.w]).as_matrix();finger=np.array([wrist.position.x,wrist.position.y,wrist.position.z])+rot@np.array([0.,0.,.166]);r['requested_finger_arm_xyz']=finger.tolist()
 for plan_only in ([True,False]if a.execute else [True]):
  goal=GraspToPoint.Goal();goal.target.header.frame_id=goal.normal.header.frame_id=goal.short_axis.header.frame_id='base_link_arm';goal.target.point.x,goal.target.point.y,goal.target.point.z=finger.tolist();goal.normal.vector.x,goal.normal.vector.y,goal.normal.vector.z=rot[:,2].tolist();goal.short_axis.vector.x,goal.short_axis.vector.y,goal.short_axis.vector.z=rot[:,1].tolist();goal.ee_link='link_6';goal.finger_link='gripper_l_link_finger';goal.plan_only=plan_only;goal.label='original_course_observation';goal.confidence=1.
  before=len(cmd);active=wait(client.send_goal_async(goal));assert active.accepted;done=wait(active.get_result_async(),90);active=None;result=done.result
  r.setdefault('actions',[]).append({'plan_only':plan_only,'status':done.status,'success':result.success,'code':result.error_code,'message':result.message,'commands':len(cmd)-before})
  if not result.success or done.status!=4:raise RuntimeError('Observation MoveIt: '+result.message)
  if plan_only:assert len(cmd)==before
 r['final_joints_rad']=list(j[-1].position);r['status']='passed'
except Exception as e:r['errors'].append(repr(e))
finally:
 if active is not None:
  try:wait(active.cancel_goal_async(),3)
  except Exception:pass
 Path('/workspace/course/reports/course-demo-stow'+('-execute'if a.execute else '-plan')+'.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
