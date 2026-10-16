#!/usr/bin/env python3
"""Plan-only check: finger target must use requested, not previous, EE rotation.

The FK-generated rotated wrist target is known reachable. No execution goal is
sent. Compares Action.commanded_pose with the independent desired link_6 FK.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from eco65b_grasp_interfaces.action import GraspToPoint
from arm_motion_check import rotation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--reject-unreachable-orientation',action='store_true',
                   help='Request reversed approach at the extended current finger position; require refusal')
    args=p.parse_args()
    rclpy.init();n=rclpy.create_node('arm_offset_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    joints,commands=[],[]
    subs=[n.create_subscription(JointState,'/arm/joint_state_deal',joints.append,qos_profile_sensor_data),
          n.create_subscription(JointState,'/hand_command',commands.append,10)]
    fk=n.create_client(GetPositionFK,'/compute_fk');client=ActionClient(n,GraspToPoint,'/grasp_to_point')
    report={'status':'failed','scope':'real plan-only finger offset geometry; no arm motion','errors':[]}

    def wait(f,timeout=20):
        rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
        if not f.done() or f.result() is None:raise RuntimeError('ROS wall timeout')
        return f.result()

    active=None
    try:
        deadline=time.monotonic()+10
        while len(joints)<10:
            if time.monotonic()>deadline:raise RuntimeError('Missing joint states')
            rclpy.spin_once(n,timeout_sec=.02)
        if not fk.wait_for_service(timeout_sec=10) or not client.wait_for_server(timeout_sec=10):raise RuntimeError('MoveIt unavailable')
        target_state=copy.deepcopy(joints[-1])
        if not args.reject_unreachable_orientation:
            target_state.position[list(target_state.name).index('joint_5')]+=.15
        q=GetPositionFK.Request();q.header.frame_id='base_link_arm';q.fk_link_names=['link_6'];q.robot_state.joint_state=target_state
        response=wait(fk.call_async(q))
        if response.error_code.val!=1:raise RuntimeError('FK failure')
        desired=response.pose_stamped[0].pose;r=rotation(desired.orientation)
        expected=[desired.position.x,desired.position.y,desired.position.z]
        goal=GraspToPoint.Goal();goal.plan_only=True;goal.ee_link='link_6';goal.finger_link='gripper_l_link_finger'
        goal.target.header.frame_id=goal.normal.header.frame_id=goal.short_axis.header.frame_id='base_link_arm'
        goal.target.point.x,goal.target.point.y,goal.target.point.z=[v+row[2]*.166 for v,row in zip(expected,r)]
        goal.normal.vector.x,goal.normal.vector.y,goal.normal.vector.z=[row[2] for row in r]
        if args.reject_unreachable_orientation:
            goal.normal.vector.x *= -1
            goal.normal.vector.y *= -1
            goal.normal.vector.z *= -1
        goal.short_axis.vector.x,goal.short_axis.vector.y,goal.short_axis.vector.z=[row[1] for row in r]
        commands.clear();active=wait(client.send_goal_async(goal))
        if not active.accepted:raise RuntimeError('Plan-only goal rejected')
        response=wait(active.get_result_async());active=None
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
        pose=response.result.commanded_pose.pose
        actual=[pose.position.x,pose.position.y,pose.position.z];error=math.dist(expected,actual)
        qa,qt=pose.orientation,desired.orientation
        angle=2*math.acos(min(1.,abs(qa.x*qt.x+qa.y*qt.y+qa.z*qt.z+qa.w*qt.w)))
        report.update(expected_ee_position=expected,commanded_ee_position=actual,position_error_m=error,
                      orientation_error_rad=angle,command_messages=len(commands),action_status=response.status,
                      action_message=response.result.message,action_success=response.result.success)
        report['expect_orientation_refusal']=args.reject_unreachable_orientation
        if args.reject_unreachable_orientation:
            if response.result.success or response.status!=6:
                report['errors'].append('Unreachable requested orientation was silently replaced by current orientation')
        else:
            if error>.0001 or angle>.01:report['errors'].append('Finger offset was not transformed by requested EE orientation')
            if not response.result.success or response.status!=4:report['errors'].append('Known reachable plan failed: '+response.result.message)
        if commands:report['errors'].append('Plan-only request sent joint commands')
        report['status']='passed' if not report['errors'] else 'failed'
    except Exception as e:report['errors'].append(str(e))
    finally:
        if active is not None:
            try:wait(active.cancel_goal_async(),3)
            except Exception:pass
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True);n.destroy_node();rclpy.shutdown()
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
