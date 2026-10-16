"""Real detected point -> original Actions -> physical lift, no attachment.

Teaching marker is at x=1.65 in an independent scene. Use a 30-degree approach,
5cm pregrasp along the tool, and 10cm vertical lift. Ground truth only verifies
physical results and places table collision geometry; it never replaces vision.
"""
import argparse,copy,json,math,time
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
from moveit_msgs.srv import ApplyPlanningScene
from simulation_interfaces.srv import GetEntityState
from eco65b_grasp_interfaces.action import GraspToPoint,GripperControl
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');a=p.parse_args()
data=json.loads(Path('/workspace/course/reports/course-demo-detect-layout.json').read_text());assert data['status']=='passed';d=data['result']
rclpy.init();n=rclpy.create_node('course_physical_grasp',parameter_overrides=[Parameter('use_sim_time',value=True)]);arm=ActionClient(n,GraspToPoint,'/grasp_to_point');gripper=ActionClient(n,GripperControl,'/gripper_control');getter=n.create_client(GetEntityState,'/get_entity_state');apply=n.create_client(ApplyPlanningScene,'/apply_planning_scene');commands=[];joint_states=[];subs=[n.create_subscription(JointState,'/hand_command',commands.append,100),n.create_subscription(JointState,'/joint_states',joint_states.append,qos_profile_sensor_data)];active=None
r={'status':'failed','scope':'physical course grasp module; real vision target, real MoveIt, real finger contact, no physical attachment','execute':a.execute,'detection_source':'reports/course-demo-detect-layout.json','plans':[],'actions':[],'object_history':[],'errors':[]}
root='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/ECO65_B'
def wait(f,timeout=60):
 end=time.monotonic()+timeout
 while not f.done():
  if time.monotonic()>end:raise RuntimeError('Course Action/service timeout')
  rclpy.spin_once(n,timeout_sec=.01)
 return f.result()
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def state(path):
 v=wait(getter.call_async(GetEntityState.Request(entity=path)),10)
 if v.result.result!=1:raise RuntimeError('SDK state: '+str(v.result))
 return v.state

def matrix(p):
 T=np.eye(4);q=p.orientation;T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[p.position.x,p.position.y,p.position.z];return T

def observe(label):
 obj=state('/World/CourseMarker');wrist=state(root+'/link_6');p=obj.pose.position;w=wrist.pose.position;sample={'stage':label,'receipt_sim_seconds':n.get_clock().now().nanoseconds*1e-9,'object_xyz':[p.x,p.y,p.z],'wrist_xyz':[w.x,w.y,w.z],'object_source_stamp':obj.header.stamp.sec+obj.header.stamp.nanosec*1e-9};r['object_history'].append(sample);return sample

def move(point,normal,axis,label,plan_only):
 global active
 g=GraspToPoint.Goal();g.target.header.frame_id=g.normal.header.frame_id=g.short_axis.header.frame_id='base_link_arm';g.target.point.x,g.target.point.y,g.target.point.z=point.tolist();g.normal.vector.x,g.normal.vector.y,g.normal.vector.z=normal.tolist();g.short_axis.vector.x,g.short_axis.vector.y,g.short_axis.vector.z=axis.tolist();g.ee_link='link_6';g.finger_link='gripper_l_link_finger';g.plan_only=plan_only;g.label=d['detected_class'];g.confidence=d['confidence'];before=len(commands);active=wait(arm.send_goal_async(g),10)
 if not active.accepted:raise RuntimeError('Arm goal rejected')
 v=wait(active.get_result_async(),90);active=None;item={'stage':label,'plan_only':plan_only,'finger_xyz':point.tolist(),'status':v.status,'success':v.result.success,'message':v.result.message,'hand_messages':len(commands)-before};r['plans'if plan_only else 'actions'].append(item)
 if v.status!=4 or not v.result.success:raise RuntimeError(label+': '+v.result.message)
 if plan_only and len(commands)!=before:raise RuntimeError('Plan-only unexpectedly sent commands')

def hand(command,position,wait_for_completion=True):
 global active
 g=GripperControl.Goal(command=command,position=float(position),speed=50.,wait_for_completion=wait_for_completion);active=wait(gripper.send_goal_async(g),10)
 if not active.accepted:raise RuntimeError('Gripper goal rejected')
 v=wait(active.get_result_async(),20);active=None;r['actions'].append({'stage':'gripper_'+command,'status':v.status,'success':v.result.success,'message':v.result.message,'measured_percent':v.result.final_position,'wait_for_completion':wait_for_completion})
 if v.status!=4 or not v.result.success:raise RuntimeError('Gripper: '+v.result.message)
try:
 assert arm.wait_for_server(timeout_sec=10)and gripper.wait_for_server(timeout_sec=10)and getter.wait_for_service(timeout_sec=10)and apply.wait_for_service(timeout_sec=10)
 # Use the actual SDK arm-base pose, avoiding authored chassis/mount assumptions.
 arm_world=matrix(state(root+'/base_link_arm').pose);geometry=json.loads(Path('/workspace/course/bobac_robotac_src/src/bobac3_navigation/maps/world_scene_geometry.json').read_text());scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True;r['arm_base_world_matrix']=arm_world.tolist()
 for i,entry in enumerate(x for x in geometry['collisions']if x['path'].startswith(('/World/table1/','/World/table2/'))):
  xyz=np.array(entry['world_points_m']);low=xyz.min(axis=0);high=xyz.max(axis=0);o=CollisionObject(id='course_demo_table_'+str(i),operation=CollisionObject.ADD);o.header.frame_id='base_link_arm';o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=(high-low).tolist())];p=Pose();p.position.x,p.position.y,p.position.z=(arm_world[:3,:3].T@((high+low)/2-arm_world[:3,3])).tolist();q=Rotation.from_matrix(arm_world[:3,:3].T).as_quat();p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q.tolist();o.primitive_poses=[p];scene.world.collision_objects.append(o)
 assert len(scene.world.collision_objects)==6 and wait(apply.call_async(ApplyPlanningScene.Request(scene=scene)),10).success
 detected=np.array([d['object_position']['point'][k]for k in 'xyz']);axis=np.array([d['long_axis']['vector'][k]for k in 'xyz']);axis/=np.linalg.norm(axis);down=np.array([0.,0.,-1.]);tilt=np.cross(down,axis);tilt/=np.linalg.norm(tilt)
 if tilt[0]>0:tilt=-tilt
 normal=down*math.cos(math.pi/6)+tilt*.5;pre=detected-normal*.08;grasp=detected-normal*.03;lift=grasp+np.array([0.,0.,.10]);r['detected_xyz']=detected.tolist();r['normal']=normal.tolist();r['long_axis']=axis.tolist();r['requested_lift_m']=.10;r['baseline']=observe('baseline')
 for label,point in [('pregrasp',pre),('grasp',grasp),('lift',lift)]:move(point,normal,axis,label,True)
 if a.execute:
  move(pre,normal,axis,'pregrasp',False)
  hand('position',70.)  # Partial opening avoids lowering the tilted jaw into the table.
  move(grasp,normal,axis,'grasp',False)
  observe('before_close')
  # Contact prevents an empty-stroke target being reached. Dispatch close,
  # then prove holding by physical lift and continued object follow-through.
  hand('close',100.,wait_for_completion=False);spin(1.5);before=observe('before_lift')
  move(lift,normal,axis,'lift',False)
  for i in range(12):spin(.2);observe('held_after_lift')
  held=[x for x in r['object_history']if x['stage']=='held_after_lift'];rise=min(x['object_xyz'][2]for x in held)-r['baseline']['object_xyz'][2];r['observed_minimum_lift_m']=rise;r['object_wrist_relative_change_m']=float(np.linalg.norm((np.array(held[-1]['object_xyz'])-held[-1]['wrist_xyz'])-(np.array(before['object_xyz'])-before['wrist_xyz'])))
  if rise<.04:raise RuntimeError('Physical object did not leave the table and remain lifted')
  if r['object_wrist_relative_change_m']>.06:raise RuntimeError('Object did not stay with gripper')
  r['status']='passed'
 else:r['status']='planned_only'
except Exception as e:r['errors'].append(repr(e))
finally:
 if active is not None:
  try:wait(active.cancel_goal_async(),3);wait(active.get_result_async(),5)
  except Exception as e:r['errors'].append('cancel: '+repr(e))
 if joint_states:
  m=joint_states[-1]
  if 'gripper_l_joint1'in m.name:r['final_gripper_angle_rad']=m.position[m.name.index('gripper_l_joint1')]
 r['hand_message_count']=len(commands);Path('/workspace/course/reports/course-demo-grasp'+('-execute'if a.execute else '-plan')+'.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']in ['passed','planned_only']else 1)
