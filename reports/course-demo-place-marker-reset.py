import copy,json,time
from pathlib import Path
import rclpy
from simulation_interfaces.srv import GetEntityState,SetEntityState
from geometry_msgs.msg import Twist
rclpy.init();n=rclpy.create_node('course_demo_marker_setup');g=n.create_client(GetEntityState,'/get_entity_state');s=n.create_client(SetEntityState,'/set_entity_state');r={'status':'failed','scope':'teaching layout setup only; no grasp or attachment','errors':[]}
def wait(f):
 rclpy.spin_until_future_complete(n,f,timeout_sec=10);assert f.done();return f.result()
try:
 assert g.wait_for_service(timeout_sec=10)and s.wait_for_service(timeout_sec=10);v=wait(g.call_async(GetEntityState.Request(entity='/World/CourseMarker')));assert v.result.result==1;r['before']=[v.state.pose.position.x,v.state.pose.position.y,v.state.pose.position.z]
 request=SetEntityState.Request(entity='/World/CourseMarker',state=copy.deepcopy(v.state));request.state.pose.position.x=1.65;request.state.twist=Twist();v=wait(s.call_async(request));assert v.result.result==1,v.result
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 v=wait(g.call_async(GetEntityState.Request(entity='/World/CourseMarker')));assert v.result.result==1;r['after']=[v.state.pose.position.x,v.state.pose.position.y,v.state.pose.position.z];assert abs(r['after'][0]-1.65)<.03 and r['after'][2]>.60;r['status']='passed'
except Exception as e:r['errors'].append(repr(e))
finally:
 Path('/workspace/course/reports/course-demo-layout-reset.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
