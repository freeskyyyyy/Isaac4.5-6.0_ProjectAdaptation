"""Functional near-desk navigation; precision is diagnostic, Nav2 handles collision."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from simulation_interfaces.srv import GetEntityState
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('course_demo_navigation',parameter_overrides=[Parameter('use_sim_time',value=True)])
r={'status':'failed','scope':'real near-desk navigation for first functional course demo; no grasp claim','goal_xy_yaw':[2.15,.6,math.pi],'trace':[],'errors':[],'feedback':[],'precision_policy':'record precision after latest user instruction; no 30mm localization/50mm tracking/0.55m conservative circle gate; retain Nav2 real-map footprint collision and bounded commands'}
arm=[];commands=[];active=None;started=time.monotonic()
subs=[n.create_subscription(JointState,'/arm/joint_state_deal',lambda m:arm.append(m),qos_profile_sensor_data),n.create_subscription(Twist,'/cmd_vel',lambda m:commands.append(m),10)]
client=ActionClient(n,NavigateToPose,'/navigate_to_pose');getter=n.create_client(GetEntityState,'/get_entity_state');zero=n.create_publisher(Twist,'/cmd_vel',10)
def wait(f,timeout=10):
 end=time.monotonic()+timeout
 while not f.done():
  if time.monotonic()>end:raise RuntimeError('ROS service/action wall timeout')
  rclpy.spin_once(n,timeout_sec=.01)
 return f.result()
def observe():
 s=wait(getter.call_async(GetEntityState.Request(entity='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link')));assert s.result.result==1
 p=s.state.pose;q=p.orientation;v=s.state.twist
 sample={'wall_seconds':time.monotonic()-started,'receipt_sim_seconds':n.get_clock().now().nanoseconds*1e-9,'world_xy':[p.position.x,p.position.y],'world_yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'v':[v.linear.x,v.linear.y,v.angular.z]};r['trace'].append(sample)
 if not all(math.isfinite(x)for x in sample['world_xy']+[sample['world_yaw']]+sample['v']):raise RuntimeError('Non-finite simulator state')
 if not arm or len(arm[-1].position)!=6 or any(not math.isfinite(x)or abs(x)>.03 for x in arm[-1].position):raise RuntimeError('Arm transport posture unavailable/changed')
 if commands:
  c=commands[-1];values=[c.linear.x,c.linear.y,c.angular.z]
  if any(not math.isfinite(x)or abs(x)>b+1e-6 for x,b in zip(values,[.08,.08,.15])):raise RuntimeError('Velocity command invalid or outside configured bounds')
 return sample
try:
 assert client.wait_for_server(timeout_sec=15)and getter.wait_for_service(timeout_sec=10)
 end=time.monotonic()+1
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 r['initial']=observe();goal=NavigateToPose.Goal();goal.behavior_tree='/workspace/course/reports/world-recovery-consistent-no-spin.xml';goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=2.15;goal.pose.pose.position.y=.6;goal.pose.pose.orientation.z=1.
 active=wait(client.send_goal_async(goal,feedback_callback=lambda m:r['feedback'].append({'wall_seconds':time.monotonic()-started,'distance_remaining':m.feedback.distance_remaining,'recoveries':m.feedback.number_of_recoveries})));assert active.accepted
 result=active.get_result_async();deadline=time.monotonic()+240;last_query=0
 while not result.done():
  rclpy.spin_once(n,timeout_sec=.02)
  if time.monotonic()>deadline:raise RuntimeError('Near-desk navigation timeout')
  if time.monotonic()-last_query>.2:observe();last_query=time.monotonic()
 response=result.result();active=None;r['action_status']=response.status;r['action_error_code']=response.result.error_code;r['action_error_message']=response.result.error_msg
 if response.status!=4 or response.result.error_code:raise RuntimeError('NavigateToPose failed')
 end=time.monotonic()+2
 while time.monotonic()<end:observe();rclpy.spin_once(n,timeout_sec=.05)
 final=r['trace'][-1];r['final_position_error_m']=math.dist(final['world_xy'],[2.15,.6]);r['final_yaw_error_rad']=abs(math.atan2(math.sin(final['world_yaw']-math.pi),math.cos(final['world_yaw']-math.pi)))
 if r['final_position_error_m']>.15 or r['final_yaw_error_rad']>.2:raise RuntimeError('Robot is not at the near-desk station')
 if math.hypot(*final['v'][:2])>.02 or abs(final['v'][2])>.03:raise RuntimeError('Robot has not stopped')
 r['status']='passed'
except Exception as e:r['errors'].append(repr(e))
finally:
 if active is not None:
  try:wait(active.cancel_goal_async(),3);wait(active.get_result_async(),5)
  except Exception as e:r['errors'].append('cancel: '+repr(e))
 for i in range(10):zero.publish(Twist());rclpy.spin_once(n,timeout_sec=.03)
 Path('/workspace/course/reports/course-demo-navigate.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items()if k not in ['trace','feedback']},indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
