"""Small empty-gripper checks only, after navigation is stopped. Never grasp."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from eco65b_grasp_interfaces.action import GripperControl
from simulation_interfaces.srv import GetEntityState
rclpy.init();node=rclpy.create_node('empty_gripper_runtime_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
r={'status':'failed','scope':'real Isaac empty-gripper bounded feedback/cancel only; no physical object grasp acceptance','joint_trace':[],'commands':[],'feedback':[],'goals':[],'errors':[]};stage='warmup';arm=[];active=None
start=time.monotonic()
def stamp(s):return s.sec+s.nanosec*1e-9
def sim():return node.get_clock().now().nanoseconds*1e-9
def joints(m):
 if 'gripper_l_joint1' in m.name:
  v=m.position[m.name.index('gripper_l_joint1')];r['joint_trace'].append({'source_sim_seconds':stamp(m.header.stamp),'receipt_sim_seconds':sim(),'wall_seconds':time.monotonic()-start,'position_rad':v,'stage':stage})
def command(m):r['commands'].append({'receipt_sim_seconds':sim(),'names':list(m.name),'positions':list(m.position),'stage':stage})
subs=[node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data),node.create_subscription(JointState,'/arm/joint_state_deal',lambda m:arm.append(m),qos_profile_sensor_data),node.create_subscription(JointState,'/hand_command',command,10)]
client=ActionClient(node,GripperControl,'/gripper_control');getter=node.create_client(GetEntityState,'/get_entity_state')
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def wait(f,timeout=15):
 end=time.monotonic()+timeout
 while not f.done():
  if time.monotonic()>=end:raise RuntimeError('Runtime probe wall timeout')
  rclpy.spin_once(node,timeout_sec=.01)
 return f.result()
def truth():
 q=wait(getter.call_async(GetEntityState.Request(entity='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link')))
 assert q.result.result==1;p=q.state.pose;v=q.state.twist;o=p.orientation
 return {'xy':[p.position.x,p.position.y],'yaw':math.atan2(2*(o.w*o.z+o.x*o.y),1-2*(o.y*o.y+o.z*o.z)),'v':[v.linear.x,v.linear.y,v.angular.z],'receipt_sim_seconds':sim(),'native_source_stamp':stamp(q.state.header.stamp)}
def send(position,speed=10.,cancel=False):
 global active,stage
 stage=('cancel' if cancel else 'position_')+str(position)
 assert arm and len(arm[-1].position)==6 and all(math.isfinite(v)and abs(v)<.03 for v in arm[-1].position)
 assert r['joint_trace'] and 0<=sim()-r['joint_trace'][-1]['source_sim_seconds']<.15
 active=wait(client.send_goal_async(GripperControl.Goal(command='position',position=float(position),speed=float(speed),wait_for_completion=True),feedback_callback=lambda m:r['feedback'].append({'state':m.feedback.state,'measured_percent':m.feedback.current_position,'receipt_sim_seconds':sim(),'stage':stage})))
 assert active.accepted
 if cancel:
  spin(.2);ack=wait(active.cancel_goal_async(),2);assert ack.goals_canceling
 response=wait(active.get_result_async());active=None
 spin(.2)
 item={'requested_percent':position,'speed_percent':speed,'stage':stage,'status':response.status,'success':response.result.success,'message':response.result.message,'measured_percent':response.result.final_position,'actual_angle_rad':r['joint_trace'][-1]['position_rad']};r['goals'].append(item)
 if cancel:assert response.status==5 and not response.result.success
 else:
  assert response.status==4 and response.result.success
  assert abs(item['actual_angle_rad']-(.78-.78*position/100))<=.012
try:
 assert client.wait_for_server(timeout_sec=10) and getter.wait_for_service(timeout_sec=10)
 spin(2)
 r['initial_truth']=truth();assert math.hypot(*r['initial_truth']['v'][:2])<.015 and abs(r['initial_truth']['v'][2])<.02
 initial=r['joint_trace'][-1]['position_rad'];assert abs(initial)<.015
 send(95.)
 send(100.)
 send(95.,speed=1.,cancel=True)
 send(100.)
 r['final_truth']=truth();assert math.dist(r['initial_truth']['xy'],r['final_truth']['xy'])<.01
 assert abs(math.atan2(math.sin(r['initial_truth']['yaw']-r['final_truth']['yaw']),math.cos(r['initial_truth']['yaw']-r['final_truth']['yaw'])))<.02
 assert max(x['position_rad'] for x in r['joint_trace'])>.025
 assert all(x['names']==['gripper_l_joint1'] and all(math.isfinite(v)and -.001<=v<=.05 for v in x['positions'])for x in r['commands'])
 assert all(abs(v)<.03 for m in arm for v in m.position)
 r['status']='passed'
except Exception as e:r['errors'].append(str(e))
finally:
 if active is not None:
  try:wait(active.cancel_goal_async(),2);wait(active.get_result_async(),3)
  except Exception as e:r['errors'].append('cancel: '+str(e))
 r['summary']={'samples':len(r['joint_trace']),'command_count':len(r['commands']),'observed_angle_min_max_rad':[min((x['position_rad']for x in r['joint_trace']),default=None),max((x['position_rad']for x in r['joint_trace']),default=None)]}
 Path('/workspace/course/reports/gripper-feedback-runtime.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items()if k not in ['joint_trace','commands','feedback']},indent=2));node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed' else 1)
