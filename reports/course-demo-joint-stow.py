"""Plan a joint goal for home; avoid solving a singular FK pose by IK."""
import copy,json,math,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.srv import GetMotionPlan
rclpy.init();n=rclpy.create_node('course_joint_stow',parameter_overrides=[Parameter('use_sim_time',value=True)]);states=[];sub=n.create_subscription(JointState,'/arm/joint_state_deal',states.append,qos_profile_sensor_data);pub=n.create_publisher(JointState,'/hand_command',10);client=n.create_client(GetMotionPlan,'/plan_kinematic_path');r={'status':'failed','scope':'real MoveIt joint-goal home with timed hand commands','errors':[]}
def sim():return n.get_clock().now().nanoseconds*1e-9
def spin(t):
 end=time.monotonic()+t
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
try:
 assert client.wait_for_service(timeout_sec=10);spin(1);assert states and len(states[-1].position)==6
 req=GetMotionPlan.Request();q=req.motion_plan_request;q.group_name='eco65_arm';q.pipeline_id='ompl';q.num_planning_attempts=5;q.allowed_planning_time=5.;q.max_velocity_scaling_factor=.3;q.max_acceleration_scaling_factor=.3;q.start_state.joint_state=copy.deepcopy(states[-1]);q.start_state.is_diff=True;constraints=Constraints()
 for name in states[-1].name:constraints.joint_constraints.append(JointConstraint(joint_name=name,position=0.,tolerance_above=.01,tolerance_below=.01,weight=1.))
 q.goal_constraints=[constraints];f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=15);assert f.done();v=f.result().motion_plan_response;r['planning_code']=v.error_code.val;assert v.error_code.val==1
 trajectory=v.trajectory.joint_trajectory;times=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points];assert times and times[-1]>0 and all(b>a for a,b in zip(times,times[1:]));assert set(trajectory.joint_names)==set(states[-1].name)
 for p in trajectory.points:
  assert all(math.isfinite(x)for x in p.positions)and all(abs(x)<=1.+1e-6 for x in p.velocities)and all(abs(x)<=1.+1e-6 for x in p.accelerations)
 r['duration_sim_seconds']=times[-1];r['points']=len(times);start_sim=sim();deadline=time.monotonic()+max(30.,times[-1]*5)
 for p,t in zip(trajectory.points,times):
  while sim()-start_sim<t:
   if time.monotonic()>deadline or sim()<start_sim:raise RuntimeError('Timed joint execution clock timeout/reset')
   rclpy.spin_once(n,timeout_sec=.005)
  m=JointState();m.header.stamp=n.get_clock().now().to_msg();m.name=list(trajectory.joint_names);m.position=list(p.positions);pub.publish(m)
 spin(1);r['measured_final_rad']=list(states[-1].position);assert all(math.isfinite(x)and abs(x)<.03 for x in states[-1].position);r['status']='passed'
except Exception as e:r['errors'].append(repr(e))
finally:
 Path('/workspace/course/reports/course-demo-joint-stow.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
