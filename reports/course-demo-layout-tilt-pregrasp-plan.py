import json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from eco65b_grasp_interfaces.action import GraspToPoint
r=json.loads(Path('/workspace/course/reports/course-demo-detect-lookat.json').read_text());assert r['status']=='passed';d=r['result'];d['object_position']['point']['x']+=.25;rclpy.init();n=rclpy.create_node('course_detected_pregrasp',parameter_overrides=[Parameter('use_sim_time',value=True)]);c=ActionClient(n,GraspToPoint,'/grasp_to_point');out={'status':'failed','scope':'hypothetical teaching marker +0.25m X, 30deg grasp, original 5cm pregrasp plus 3cm finger offset along tool axis; plan-only','errors':[]}
try:
 assert c.wait_for_server(timeout_sec=10);g=GraspToPoint.Goal();g.plan_only=True;g.label=d['detected_class'];g.confidence=d['confidence'];g.ee_link='link_6';g.finger_link='gripper_l_link_finger'
 for field,source,key in [('target','object_position','point'),('normal','normal','vector'),('short_axis','long_axis','vector')]:
  message=getattr(g,field);message.header.frame_id=d[source]['header']['frame_id'];v=getattr(message,key)
  for axis in 'xyz':setattr(v,axis,d[source][key][axis])
 g.normal.vector.x=-.5;g.normal.vector.y=0.;g.normal.vector.z=-0.8660254037844386;g.target.point.x-=g.normal.vector.x*.08;g.target.point.z-=g.normal.vector.z*.08;out['finger_target_arm_xyz']=[g.target.point.x,g.target.point.y,g.target.point.z];f=c.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result();assert h and h.accepted;f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=45);assert f.done();v=f.result();out.update(action_status=v.status,success=v.result.success,code=v.result.error_code,message=v.result.message);out['status']='passed'if v.status==4 and v.result.success else 'failed'
except Exception as e:out['errors'].append(repr(e))
finally:
 Path('/workspace/course/reports/course-demo-layout-tilt-pregrasp-plan.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if out['status']=='passed'else 1)
