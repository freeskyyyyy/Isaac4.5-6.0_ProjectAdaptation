import rclpy,json,math,time,yaml,hashlib
from pathlib import Path
from rclpy.parameter import Parameter
from simulation_interfaces.srv import GetEntityState
from sensor_msgs.msg import JointState
root=Path('/workspace/course');prefix='course-demo'
rclpy.init();n=rclpy.create_node('staging_retry_initial_pose',parameter_overrides=[Parameter('use_sim_time',value=True)])
j=[];sub=n.create_subscription(JointState,'/arm/joint_state_deal',lambda m:j.append(m),10)
c=n.create_client(GetEntityState,'/get_entity_state');assert c.wait_for_service(timeout_sec=10)
f=c.call_async(GetEntityState.Request(entity='/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link'))
rclpy.spin_until_future_complete(n,f,timeout_sec=10);r=f.result();assert r.result.result==1
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
assert j and len(j[-1].position)==6 and all(math.isfinite(x) and abs(x)<=.02 for x in j[-1].position)
p=r.state.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
assert all(math.isfinite(x) for x in [p.position.x,p.position.y,yaw])
source=root/'bobac_robotac_src/src/bobac3_navigation/params/bobac3_navigation_world_rpp_params.yaml'
s=yaml.safe_load(source.read_text());s['amcl']['ros__parameters']['initial_pose']={'x':p.position.x,'y':p.position.y,'z':0.,'yaw':yaw}
for prefix_costmap in ['local','global']:
 cost=s[prefix_costmap+'_costmap'][prefix_costmap+'_costmap']['ros__parameters']
 cost.pop('robot_radius',None)
 cost['footprint']='[[0.35,0.23],[0.35,-0.23],[-0.35,-0.23],[-0.35,0.23]]'
 cost['footprint_padding']=.02
runtime=root/'reports'/(prefix+'-runtime.yaml');runtime.write_text(yaml.safe_dump(s,sort_keys=False))
scene=root/'scenes/test_scene_bobac_6.0.radius-candidate.usda'
assert hashlib.sha256(scene.read_bytes()).hexdigest()=='8cae3c282e9446309b18d864fd6b9f6bf1d9efa3d3e82813ad287eed31847003'
report={'initial_true_xy_yaw':[p.position.x,p.position.y,yaw],'goal_xy_yaw':[2.35,.6,math.pi],'arm_position':list(j[-1].position),'source_stamp':[r.state.header.stamp.sec,r.state.header.stamp.nanosec],'initialization':'one-time true initial pose, subsequent AMCL localization; no continuously published truth localization','scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'source_params_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'runtime_params_sha256':hashlib.sha256(runtime.read_bytes()).hexdigest()}
(root/'reports'/(prefix+'-initial.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));n.destroy_node();rclpy.shutdown()
