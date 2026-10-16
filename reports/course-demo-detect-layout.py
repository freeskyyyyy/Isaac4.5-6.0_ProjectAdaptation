import json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from eco65b_grasp_interfaces.action import DetectObject
from rosidl_runtime_py.convert import message_to_ordereddict
rclpy.init();n=rclpy.create_node('course_positive_detection',parameter_overrides=[Parameter('use_sim_time',value=True)]);c=ActionClient(n,DetectObject,'/detect_object');r={'status':'failed','scope':'real original camera and DetectObject Action; no substituted ground-truth target','feedback':[],'errors':[]};active=None
try:
 assert c.wait_for_server(timeout_sec=10);f=c.send_goal_async(DetectObject.Goal(target_classes=['pen','pencil'],confidence_threshold=.01,timeout=10.),feedback_callback=lambda m:r['feedback'].append({'state':m.feedback.state,'attempts':m.feedback.detection_attempts}));rclpy.spin_until_future_complete(n,f,timeout_sec=5);active=f.result();assert active and active.accepted;result=active.get_result_async();rclpy.spin_until_future_complete(n,result,timeout_sec=15);assert result.done();done=result.result();active=None;r['action_status']=done.status;r['result']=message_to_ordereddict(done.result);r['status']='passed'if done.status==4 and done.result.success else 'failed'
except Exception as e:r['errors'].append(repr(e))
finally:
 if active is not None:
  f=active.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=3)
 Path('/workspace/course/reports/course-demo-detect-layout.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
