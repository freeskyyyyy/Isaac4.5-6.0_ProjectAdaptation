import json,time,math
from pathlib import Path
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from tf2_ros import Buffer,TransformListener
from simulation_interfaces.srv import GetEntityState
rclpy.init();n=rclpy.create_node('course_camera_axes_probe',parameter_overrides=[Parameter('use_sim_time',value=True)]);tf=Buffer(node=n);listener=TransformListener(tf,n);messages={};qos=QoSProfile(depth=3,reliability=ReliabilityPolicy.RELIABLE)
subs=[n.create_subscription(kind,topic,lambda m,t=topic:messages.update({t:m}),qos)for kind,topic in [(Image,'/arm_camera/rgb'),(Image,'/arm_camera/depth'),(CameraInfo,'/arm_camera/camera_info')]];c=n.create_client(GetEntityState,'/get_entity_state');r={'status':'failed','errors':[]}
def wait(f):
 end=time.monotonic()+10
 while not f.done():
  if time.monotonic()>end:raise RuntimeError('SDK query timeout')
  rclpy.spin_once(n,timeout_sec=.01)
 return f.result()
def matrix(p):
 t=np.eye(4);t[:3,3]=[p.position.x,p.position.y,p.position.z];q=p.orientation;t[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();return t
try:
 assert c.wait_for_service(timeout_sec=10);end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 assert all(t in messages for t in ['/arm_camera/rgb','/arm_camera/depth','/arm_camera/camera_info'])
 rgb=messages['/arm_camera/rgb'];depth=messages['/arm_camera/depth'];info=messages['/arm_camera/camera_info'];capture=Time.from_msg(rgb.header.stamp)
 end=time.monotonic()+1.5
 while not tf.can_transform('base_link_arm',rgb.header.frame_id,capture)and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 frame=tf.lookup_transform('base_link_arm',rgb.header.frame_id,capture).transform;T=np.eye(4);T[:3,3]=[frame.translation.x,frame.translation.y,frame.translation.z];q=frame.rotation;T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
 paths={'arm':'/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/ECO65_B/base_link_arm','camera':'/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/ECO65_B/link_6/arm_camera/arm_Camera','object':'/World/CourseMarker'};world={}
 for key,path in paths.items():
  s=wait(c.call_async(GetEntityState.Request(entity=path)));assert s.result.result==1,(key,s.result);world[key]=matrix(s.state.pose)
 sdk=np.linalg.inv(world['arm'])@world['camera'];flip=np.diag([1.,-1.,-1.]);err_usd=Rotation.from_matrix(sdk[:3,:3].T@T[:3,:3]).magnitude();err_optical=Rotation.from_matrix((sdk[:3,:3]@flip).T@T[:3,:3]).magnitude();r['axis_rotation_errors_rad']={'TF_matches_USD':err_usd,'TF_matches_USD_to_optical':err_optical};r['convention']='usd'if err_usd<.05 else 'optical'if err_optical<.05 else 'unresolved';object_arm=np.linalg.inv(world['arm'])@world['object'][:,3];point=np.linalg.inv(T)@object_arm;r['object_in_ros_camera']=point[:3].tolist();r['object_in_arm']=object_arm[:3].tolist();r['camera_arm_matrix']=T.tolist();r['sdk_camera_arm_matrix']=sdk.tolist();r['sdk_world']={k:v.tolist()for k,v in world.items()};r['rgb_frame']=rgb.header.frame_id;r['K']=list(info.k);r['image_size']=[rgb.width,rgb.height];r['projections']={}
 for name,rot in [('optical',np.eye(3)),('usd',flip)]:
  o=rot@point[:3];pixel=[info.k[0]*o[0]/o[2]+info.k[2],info.k[4]*o[1]/o[2]+info.k[5]];r['projections'][name]={'z':o[2],'pixel':pixel,'visible':bool(o[2]>0 and 0<=pixel[0]<rgb.width and 0<=pixel[1]<rgb.height)}
 bridge=CvBridge();cv2.imwrite('/workspace/course/reports/course-demo-camera-calibration.png',bridge.imgmsg_to_cv2(rgb,'bgr8'));r['status']='passed'
except Exception as e:r['errors'].append(repr(e))
finally:
 Path('/workspace/course/reports/course-demo-camera-calibration.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if r['status']=='passed'else 1)
