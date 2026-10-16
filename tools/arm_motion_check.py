#!/usr/bin/env python3
"""Small real MoveIt Action trajectory, FK/TF feedback, then return to baseline.

Uses a FK-generated +0.05rad joint_1 target, never a visual grasp target. The
course server must use its default fixed 0.166m end-effector offset. No other
motion/gripper task may run during this check.
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


def rotation(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--expect-invalid-timing',action='store_true',
                   help='Negative test with missing MoveIt limits: reject plan and publish no commands')
    args=p.parse_args()
    rclpy.init()
    n=rclpy.create_node('arm_motion_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    joints,commands=[],[]
    subs=[n.create_subscription(JointState,'/arm/joint_state_deal',joints.append,qos_profile_sensor_data),
          n.create_subscription(JointState,'/hand_command',commands.append,10)]
    client=ActionClient(n,GraspToPoint,'/grasp_to_point')
    fk=n.create_client(GetPositionFK,'/compute_fk')
    report={'status':'failed','scope':'real MoveIt small arm Action trajectory and measured FK; not grasp acceptance',
            'checks':[],'errors':[]}
    active=None

    def until(pred,timeout=15):
        deadline=time.monotonic()+timeout
        while not pred():
            if time.monotonic()>deadline:raise RuntimeError('Wall-clock request timeout')
            rclpy.spin_once(n,timeout_sec=.02)

    def wait(f,timeout=30):
        until(f.done,timeout)
        return f.result()

    def pose(state):
        req=GetPositionFK.Request()
        req.header.frame_id='base_link_arm';req.fk_link_names=['link_6'];req.robot_state.joint_state=state
        res=wait(fk.call_async(req))
        if res.error_code.val!=1:raise RuntimeError('FK failure '+str(res.error_code.val))
        return res.pose_stamped[0]

    def request(target,only):
        nonlocal active
        target_rotation=rotation(target.pose.orientation)
        g=GraspToPoint.Goal();g.plan_only=only;g.ee_link='link_6';g.finger_link='link_6'
        g.target.header.frame_id='base_link_arm'
        g.target.point.x=target.pose.position.x+target_rotation[0][2]*.166
        g.target.point.y=target.pose.position.y+target_rotation[1][2]*.166
        g.target.point.z=target.pose.position.z+target_rotation[2][2]*.166
        g.normal.header.frame_id=g.short_axis.header.frame_id='base_link_arm'
        g.normal.vector.x,g.normal.vector.y,g.normal.vector.z=[r[2] for r in target_rotation]
        g.short_axis.vector.x,g.short_axis.vector.y,g.short_axis.vector.z=[r[1] for r in target_rotation]
        count=len(commands);start=time.monotonic()
        active=wait(client.send_goal_async(g))
        if not active.accepted:raise RuntimeError('Arm goal rejected')
        res=wait(active.get_result_async(),30)
        active=None
        end=time.monotonic()+.3
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        check={'plan_only':only,'status':res.status,'success':res.result.success,'message':res.result.message,
               'command_messages':len(commands)-count,'wall_seconds':time.monotonic()-start}
        if not only and len(commands)>count:
            stamps=[m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in commands[count:]]
            check['command_stamp_span_sim_seconds']=stamps[-1]-stamps[0]
            if stamps[-1]-stamps[0]<.01:
                report['errors'].append('Nonzero arm trajectory published with zero/near-zero simulation duration')
        report['checks'].append(check)
        if args.expect_invalid_timing:
            if res.status!=6 or res.result.success or 'Invalid trajectory timing' not in res.result.message or len(commands)!=count:
                raise RuntimeError('Invalid timing was not rejected without commands')
            return
        if res.status!=4 or not res.result.success:raise RuntimeError('Arm Action failed: '+res.result.message)
        if only and len(commands)!=count:raise RuntimeError('Integrated plan_only published joint commands')
        if not only and len(commands)==count:raise RuntimeError('Execution returned without joint commands')
        if not only:
            deadline=time.monotonic()+5
            while True:
                actual=pose(joints[-1]);distance=math.dist(
                    [actual.pose.position.x,actual.pose.position.y,actual.pose.position.z],
                    [target.pose.position.x,target.pose.position.y,target.pose.position.z])
                qa=actual.pose.orientation;qt=target.pose.orientation
                angle=2*math.acos(min(1.,abs(qa.x*qt.x+qa.y*qt.y+qa.z*qt.z+qa.w*qt.w)))
                check.update(position_error_m=distance,orientation_error_rad=angle,
                             actual_joints=dict(zip(joints[-1].name,joints[-1].position)))
                if distance<.005 and angle<.02:break
                if time.monotonic()>deadline:raise RuntimeError('Actual arm FK did not reach commanded pose')
                rclpy.spin_once(n,timeout_sec=.05)

    try:
        until(lambda:len(joints)>10)
        if not fk.wait_for_service(timeout_sec=10) or not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('MoveIt FK or arm Action unavailable')
        peers=n.get_publishers_info_by_topic('/hand_command')
        if any(i.node_name not in ('eco65b_grasp_moveit_node','gripper_control_node') for i in peers):
            raise RuntimeError('Unexpected joint command publisher')
        baseline=copy.deepcopy(joints[-1]);target_state=copy.deepcopy(baseline)
        target_state.position[list(target_state.name).index('joint_1')]+=.05
        report['baseline_joints']=dict(zip(baseline.name,baseline.position))
        target=pose(target_state)
        request(target,True)
        if not args.expect_invalid_timing:
            request(target,False)
            request(pose(baseline),False)
        report['status']='passed' if not report['errors'] else 'failed'
    except Exception as e:
        report['errors'].append(str(e))
    finally:
        if active is not None:
            try:wait(active.cancel_goal_async(),3)
            except Exception as e:report['errors'].append('Cleanup cancellation: '+str(e))
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
        n.destroy_node();rclpy.shutdown()
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
