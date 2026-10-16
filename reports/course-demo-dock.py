"""Small supervised final desk approach after Nav2, using the validated smoother."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from simulation_interfaces.srv import GetEntityState
rclpy.init();n=rclpy.create_node('course_final_desk_approach',parameter_overrides=[Parameter('use_sim_time',value=True)]);pub=n.create_publisher(Twist,'/cmd_vel_nav',10);states=[];sub=n.create_subscription(JointState,'/arm/joint_state_deal',states.append,qos_profile_sensor_data);c=n.create_client(GetEntityState,'/get_entity_state');r={'status':'failed','scope':'small final desk approach for fixed-base grasp module, after prior Nav2 arrival','trace':[],'errors':[]}
def sample():
 f=c.call_async(GetEntityState.Request(entity='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link'));rclpy.spin_until_future_complete(n,f,timeout_sec=3);assert f.done();v=f.result();assert v.result.result==1;p=v.state.pose;q=p.orientation;t=v.state.twist;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));row={'receipt_sim_seconds':n.get_clock().now().nanoseconds*1e-9,'xy':[p.position.x,p.position.y],'yaw':yaw,'v':[t.linear.x,t.linear.y,t.angular.z]};r['trace'].append(row);return row
try:
 assert c.wait_for_service(timeout_sec=10);end=time.monotonic()+1
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 assert states and len(states[-1].position)==6 and all(math.isfinite(x)and abs(x)<.03 for x in states[-1].position)
 initial=sample();r['initial']=initial;deadline=time.monotonic()+30
 while True:
  row=sample()
  if not all(math.isfinite(x)for x in row['xy']+[row['yaw']]):raise RuntimeError('Invalid base pose')
  if abs(math.atan2(math.sin(row['yaw']-math.pi),math.cos(row['yaw']-math.pi)))>.2:raise RuntimeError('Not facing desk')
  if any(abs(x)>.03 for x in states[-1].position):raise RuntimeError('Arm left stow posture')
  if row['xy'][0]<=2.04:break
  if time.monotonic()>deadline or math.dist(initial['xy'],row['xy'])>.35:raise RuntimeError('Docking wall/displacement bound')
  msg=Twist();msg.linear.x=.03;pub.publish(msg);rclpy.spin_once(n,timeout_sec=.04)
 for i in range(30):pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
 r['final']=sample();assert math.hypot(*r['final']['v'][:2])<.02;r['status']='passed'
except Exception as e:r['errors'].append(repr(e))
finally:
 for i in range(10):pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
 Path('/workspace/course/reports/course-demo-dock.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items()if k!='trace'},indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
