import math
import os
from typing import Optional
import time
from ament_index_python.packages import get_package_share_directory
import shutil

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Vector3Stamped

from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from eco65b_grasp_interfaces.action import DetectObject

from ultralytics import YOLOE

try:
    import cv2
except Exception:
    cv2 = None


class YoloEGraspNode(Node):
    def __init__(self):
        super().__init__('yoloe_grasp_node')

        self.declare_parameter('model_path', 'yoloe-26m-seg.pt')
        self.declare_parameter('classes', ['pencil', 'pen'])
        self.declare_parameter('conf', 0.01)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('debug_image_dir', '/workspace/bobac_ws/yoloe_grasp_debug')

        self.declare_parameter('rgb_topic', '/arm_camera/rgb')
        self.declare_parameter('depth_topic', '/arm_camera/depth')
        self.declare_parameter('camera_info_topic', '/arm_camera/camera_info')
        self.declare_parameter('base_frame', 'base_link_arm')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('disable_optical_compensation', False)

        # Grasp orientation settings
        self.declare_parameter('normal_mode', 'local_plane')
        self.declare_parameter('fixed_normal', [0.0, 0.0, 1.0])
        self.declare_parameter('axis_mode', 'mask_long')
        self.declare_parameter('axis_step_px', 10)
        self.declare_parameter('local_plane_radius_px', 12)
        self.declare_parameter('local_plane_depth_tol', 0.015)
        self.declare_parameter('local_plane_min_points', 80)

        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame_override = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.disable_optical_compensation = bool(self.get_parameter('disable_optical_compensation').value)

        self.normal_mode = self.get_parameter('normal_mode').get_parameter_value().string_value
        self.fixed_normal = [float(v) for v in self.get_parameter('fixed_normal').value]
        self.axis_mode = self.get_parameter('axis_mode').get_parameter_value().string_value
        self.axis_step_px = int(self.get_parameter('axis_step_px').value)
        self.local_plane_radius_px = int(self.get_parameter('local_plane_radius_px').value)
        self.local_plane_depth_tol = float(self.get_parameter('local_plane_depth_tol').value)
        self.local_plane_min_points = int(self.get_parameter('local_plane_min_points').value)
        
        classes_param = self.get_parameter('classes').get_parameter_value().string_array_value
        self.classes = [str(v) for v in classes_param]
        self.conf = float(self.get_parameter('conf').value)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.save_debug_images = bool(self.get_parameter('save_debug_images').value)
        self.debug_image_dir = self.get_parameter('debug_image_dir').get_parameter_value().string_value

        # Action Server 替代 Action Client
        self._action_server = ActionServer(
            self,
            DetectObject,
            '/detect_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.bridge = CvBridge()
        self._last_rgb: Optional[Image] = None
        self._last_depth: Optional[Image] = None
        self._camera_info: Optional[CameraInfo] = None
        self._camera_frame: Optional[str] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, qos_profile_sensor_data)

        self.model_path_file_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.pkg_share = get_package_share_directory('yoloe_grasp')
        self.model_path = os.path.join(f"{self.pkg_share}", f"weights/{self.model_path_file_path}")

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f'课程 YOLOE 权重不存在: {self.model_path}')
        # Ultralytics resolves the MobileCLIP TorchScript checkpoint from cwd.
        # Use a writable cache rather than inferring a colcon install layout.
        self.declare_parameter('text_model_cache_dir', '/workspace/course/.runtime/vision/weights')
        text_cache = os.path.abspath(self.get_parameter('text_model_cache_dir').value)
        os.makedirs(text_cache, exist_ok=True)
        text_source = os.path.join(self.pkg_share, 'weights', 'mobileclip2_b.ts')
        text_target = os.path.join(text_cache, 'mobileclip2_b.ts')
        if not os.path.isfile(text_target):
            if not os.path.isfile(text_source):
                raise FileNotFoundError(f'课程 MobileCLIP 权重不存在: {text_source}')
            shutil.copy2(text_source, text_target)
        os.chdir(text_cache)

        self.model = YOLOE(self.model_path)
        if self.classes:
            self.model.set_classes(self.classes)

        self._logged_axes = False
        self._throttle_last = {}
        self._saved_debug = False

        self.get_logger().info('YOLO 检测节点已启动（Action Server 模式）')
        self.get_logger().info(f'模型: {self.model_path}, 设备: {self.device}')
        self.get_logger().info(f'Action Server: /detect_object')

    def goal_callback(self, goal_request):
        """处理新的检测请求"""
        self.get_logger().info(f'收到检测请求: classes={goal_request.target_classes}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """处理取消请求"""
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """执行物体检测"""
        self.get_logger().info('开始执行物体检测...')

        goal = goal_handle.request
        feedback_msg = DetectObject.Feedback()
        result = DetectObject.Result()

        try:
            # 设置检测参数
            target_classes = goal.target_classes if goal.target_classes else self.classes
            conf_threshold = goal.confidence_threshold if goal.confidence_threshold > 0 else self.conf
            timeout = goal.timeout if goal.timeout > 0 else 30.0

            if target_classes:
                self.model.set_classes(target_classes)

            # 等待检测
            feedback_msg.state = 'WAITING_IMAGE'
            feedback_msg.detection_attempts = 0
            goal_handle.publish_feedback(feedback_msg)

            start_time = self.get_clock().now()
            detection_attempts = 0

            while True:
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    result.success = False
                    result.message = f'检测超时（{timeout}秒）'
                    self.get_logger().error(result.message)
                    goal_handle.abort()
                    return result

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = '已取消'
                    self.get_logger().warn(result.message)
                    return result

                # 尝试检测
                feedback_msg.state = 'DETECTING'
                detection_attempts += 1
                feedback_msg.detection_attempts = detection_attempts
                goal_handle.publish_feedback(feedback_msg)

                detection_result = self._tick_once(conf_threshold)

                if detection_result is not None:
                    feedback_msg.state = 'FOUND'
                    goal_handle.publish_feedback(feedback_msg)

                    result.success = True
                    result.message = '检测成功'
                    result.detected_class = detection_result['label']
                    result.object_position = detection_result['position']
                    result.normal = detection_result['normal']
                    result.long_axis = detection_result['long_axis']
                    result.confidence = detection_result['confidence']

                    self.get_logger().info(f'✓ 检测成功: {result.detected_class} (置信度: {result.confidence:.3f})')
                    goal_handle.succeed()
                    return result
                else:
                    # 记录检测失败的原因（通过查看最近的警告日志）
                    if detection_attempts == 1:
                        self.get_logger().info('首次检测未找到目标，继续尝试...')

                time.sleep(0.1)

        except Exception as e:
            result.success = False
            result.message = f'检测过程中发生异常: {str(e)}'
            self.get_logger().error(result.message)
            import traceback
            self.get_logger().error(f'异常堆栈:\n{traceback.format_exc()}')
            goal_handle.abort()
            return result

    @staticmethod
    def _normalize(v: np.ndarray) -> Optional[np.ndarray]:
        n = float(np.linalg.norm(v))
        if not np.isfinite(n) or n < 1e-9:
            return None
        return v / n

    def _tick_once(self, conf_threshold):
        """执行一次检测，返回检测结果或 None"""
        # 直接调用 _tick 方法（它现在返回检测结果）
        return self._tick()

    @staticmethod
    def _pca_2d_axis(mask: np.ndarray, which: str) -> Optional[np.ndarray]:
        ys, xs = np.where(mask)
        if xs.size < 30:
            return None
        pts = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
        mu = pts.mean(axis=0)
        pts = pts - mu
        cov = (pts.T @ pts) / float(pts.shape[0])
        w, v = np.linalg.eigh(cov)
        if which == 'short':
            axis = v[:, int(np.argmin(w))]
        else:
            axis = v[:, int(np.argmax(w))]
        n = float(np.linalg.norm(axis))
        if not np.isfinite(n) or n < 1e-9:
            return None
        return axis / n

    def _backproject(self, u: float, v: float, z: float) -> np.ndarray:
        fx = float(self._camera_info.k[0])
        fy = float(self._camera_info.k[4])
        cx = float(self._camera_info.k[2])
        cy = float(self._camera_info.k[5])
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return np.array([x, y, z], dtype=np.float64)

    def _estimate_local_plane_normal_base(self, depth: np.ndarray, mask: np.ndarray, u0: int, v0: int, z0: float,
                                          tf_cam_to_base) -> Optional[np.ndarray]:
        if self._camera_info is None:
            return None
        if mask is None or mask.shape[0] != depth.shape[0] or mask.shape[1] != depth.shape[1]:
            return None

        r = max(2, int(self.local_plane_radius_px))
        umin = max(0, u0 - r)
        umax = min(depth.shape[1] - 1, u0 + r)
        vmin = max(0, v0 - r)
        vmax = min(depth.shape[0] - 1, v0 + r)

        # Collect local mask-constrained points with similar depth (to reject background/table).
        pts_cam = []
        tol = max(0.001, float(self.local_plane_depth_tol))
        for vv in range(vmin, vmax + 1):
            for uu in range(umin, umax + 1):
                if not mask[vv, uu]:
                    continue
                zz = float(depth[vv, uu])
                if not np.isfinite(zz) or zz <= 0.0:
                    continue
                if abs(zz - z0) > tol:
                    continue
                pts_cam.append(self._backproject(float(uu), float(vv), zz))

        if len(pts_cam) < int(self.local_plane_min_points):
            return None

        P = np.stack(pts_cam, axis=0)

        # Transform to base_frame using tf (rotation + translation).
        q = tf_cam_to_base.transform.rotation
        t = tf_cam_to_base.transform.translation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        tx, ty, tz = float(t.x), float(t.y), float(t.z)

        def rot_only(v: np.ndarray) -> np.ndarray:
            vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
            t0 = 2.0 * (qy * vz - qz * vy)
            t1 = 2.0 * (qz * vx - qx * vz)
            t2 = 2.0 * (qx * vy - qy * vx)
            rx = vx + qw * t0 + (qy * t2 - qz * t1)
            ry = vy + qw * t1 + (qz * t0 - qx * t2)
            rz = vz + qw * t2 + (qx * t1 - qy * t0)
            return np.array([rx, ry, rz], dtype=np.float64)

        # Apply rotation and translation.
        P_base = np.empty_like(P)
        for i in range(P.shape[0]):
            rb = rot_only(P[i])
            P_base[i, 0] = rb[0] + tx
            P_base[i, 1] = rb[1] + ty
            P_base[i, 2] = rb[2] + tz

        mu = P_base.mean(axis=0)
        Q = P_base - mu
        C = (Q.T @ Q) / float(Q.shape[0])
        w, v = np.linalg.eigh(C)
        n = v[:, int(np.argmin(w))]
        n = self._normalize(n)
        if n is None:
            return None

        # Choose a consistent sign: point the normal toward the camera position (in base_frame).
        cam_pos = np.array([tx, ty, tz], dtype=np.float64)
        view = self._normalize(cam_pos - mu)
        if view is not None and float(np.dot(n, view)) < 0.0:
            n = -n
        return n

    def _on_rgb(self, msg: Image):
        self._last_rgb = msg
        if msg.header.frame_id:
            self._camera_frame = msg.header.frame_id

    def _on_depth(self, msg: Image):
        self._last_depth = msg

    def _on_camera_info(self, msg: CameraInfo):
        self._camera_info = msg
        if msg.header.frame_id:
            self._camera_frame = msg.header.frame_id

    def _warn_throttle(self, key: str, period_ms: int, msg: str):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._throttle_last.get(key)
        if last_ns is None or (now_ns - last_ns) >= period_ms * 1_000_000:
            self._throttle_last[key] = now_ns
            self.get_logger().warn(msg)

    def _tick(self):
        if self._last_rgb is None:
            self._warn_throttle('no_rgb', 2000, '等待 RGB 图像...')
            return None
        if self._last_depth is None:
            self._warn_throttle('no_depth', 2000, '等待深度图像...')
            return None
        if self._camera_info is None:
            self._warn_throttle('no_camera_info', 2000, '等待相机信息...')
            return None

        camera_frame = self.camera_frame_override or self._camera_frame
        if not camera_frame:
            self._warn_throttle('no_camera_frame', 2000, '相机坐标系未知')
            return None

        # Prefer an optical frame if one exists in TF (backprojection assumes optical convention).
        optical_frame = ''
        if camera_frame and camera_frame != self.base_frame:
            candidates = [
                f"{camera_frame}_optical_frame",
                f"{camera_frame}/optical_frame",
                "camera_color_optical_frame",
                "camera_depth_optical_frame",
            ]
            for c in candidates:
                try:
                    self.tf_buffer.lookup_transform(self.base_frame, c, rclpy.time.Time())
                    optical_frame = c
                    break
                except TransformException:
                    continue
        tf_frame = optical_frame or camera_frame

        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(self._last_rgb, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge rgb convert failed: {e}')
            return None

        try:
            depth = self.bridge.imgmsg_to_cv2(self._last_depth)
            if self._last_depth.encoding == '16UC1':
                depth = depth.astype(np.float32) * 0.001
            elif self._last_depth.encoding != '32FC1':
                raise ValueError(f'Unsupported depth encoding: {self._last_depth.encoding}')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge depth convert failed: {e}')
            return None

        results = self.model.predict(cv_rgb, conf=self.conf, device=self.device, verbose=False)
        if not results:
            return None

        r0 = results[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return None

        confs = r0.boxes.conf.detach().cpu().numpy()
        best_i = int(np.argmax(confs))
        best_conf = float(confs[best_i])
        x1, y1, x2, y2 = r0.boxes.xyxy[best_i].detach().cpu().numpy().tolist()

        u = int(round((x1 + x2) * 0.5))
        v = int(round((y1 + y2) * 0.5))

        if v < 0 or u < 0 or v >= depth.shape[0] or u >= depth.shape[1]:
            self.get_logger().warn(f'检测中心点超出深度图范围: (u,v)=({u},{v}), depth.shape={depth.shape}')
            return None

        z_single = float(depth[v, u])  # 32FC1 meters

        z = z_single
        # Prefer depth estimated from the segmentation mask (more robust than bbox center).
        try:
            if getattr(r0, 'masks', None) is not None and r0.masks is not None and r0.masks.data is not None:
                m = r0.masks.data[best_i].detach().cpu().numpy()
                if m is not None:
                    # m is (H, W) mask aligned to the input image resolution
                    mask = m > 0.5
                    # If mask resolution doesn't match depth (common), resize it.
                    if cv2 is not None and (mask.shape[0] != depth.shape[0] or mask.shape[1] != depth.shape[1]):
                        mask = cv2.resize(mask.astype(np.uint8), (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
                    if mask.shape[0] == depth.shape[0] and mask.shape[1] == depth.shape[1]:
                        vals = depth[mask]
                        vals = vals[np.isfinite(vals)]
                        vals = vals[vals > 0.0]
                        if vals.size > 0:
                            z = float(np.median(vals))
        except Exception:
            pass

        if math.isnan(z) or z <= 0.0:
            self._warn_throttle('depth_invalid', 2000, f'yoloe: depth invalid at (u,v)=({u},{v}) z={z}')
            return None

        fx = float(self._camera_info.k[0])
        fy = float(self._camera_info.k[4])
        cx = float(self._camera_info.k[2])
        cy = float(self._camera_info.k[5])

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        p_cam = PointStamped()
        p_cam.header = self._last_rgb.header
        p_cam.header.frame_id = tf_frame
        p_cam.point.x = float(x)
        p_cam.point.y = float(y)
        p_cam.point.z = float(z)

        # If TF doesn't provide an optical frame, compensate by rotating the point from the
        # camera's non-optical frame into an optical-style convention.
        # RotX(-90deg): (x, y, z) -> (x, z, -y)
        # NOTE: Disable this if your camera frame is already in optical convention
        if not optical_frame and camera_frame and tf_frame == camera_frame and not self.disable_optical_compensation:
            p_cam.point.x, p_cam.point.y, p_cam.point.z = (
                p_cam.point.x,
                p_cam.point.z,
                -p_cam.point.y,
            )

        axis_img = None
        mask_for_axes = None
        try:
            if getattr(r0, 'masks', None) is not None and r0.masks is not None and r0.masks.data is not None:
                m = r0.masks.data[best_i].detach().cpu().numpy()
                if m is not None:
                    mask_for_axes = (m > 0.5)
                    if cv2 is not None and (mask_for_axes.shape[0] != depth.shape[0] or mask_for_axes.shape[1] != depth.shape[1]):
                        mask_for_axes = cv2.resize(
                            mask_for_axes.astype(np.uint8),
                            (depth.shape[1], depth.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        ).astype(bool)
        except Exception:
            mask_for_axes = None

        if mask_for_axes is not None and mask_for_axes.shape[0] == depth.shape[0] and mask_for_axes.shape[1] == depth.shape[1]:
            which = 'long' if str(self.axis_mode).lower() == 'mask_long' else 'short'
            axis_img = self._pca_2d_axis(mask_for_axes, which)

        # Default to image x-axis if PCA fails
        if axis_img is None:
            axis_img = np.array([1.0, 0.0], dtype=np.float64)

        step = max(2, int(self.axis_step_px))
        u2 = float(np.clip(u + axis_img[0] * step, 0, depth.shape[1] - 1))
        v2 = float(np.clip(v + axis_img[1] * step, 0, depth.shape[0] - 1))
        z2 = float(depth[int(round(v2)), int(round(u2))])
        if not np.isfinite(z2) or z2 <= 0.0:
            z2 = float(z)

        p2_cam = self._backproject(u2, v2, z2)
        p1_cam = np.array([p_cam.point.x, p_cam.point.y, p_cam.point.z], dtype=np.float64)
        axis_cam = self._normalize(p2_cam - p1_cam)
        if axis_cam is None:
            axis_cam = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        # Estimate normal
        n_cam = None
        n_base_direct = None
        mode = str(self.normal_mode).lower()
        if mode == 'camera_side':
            # Will be computed in base_frame after we have object point in base_frame.
            pass
        elif mode == 'local_plane':
            # Will be computed from a local depth patch around the grasp point.
            pass
        elif mode == 'fixed_base':
            if len(self.fixed_normal) == 3:
                n_base_direct = np.array(self.fixed_normal, dtype=np.float64)
            else:
                n_base_direct = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            n_base_direct = self._normalize(n_base_direct)
            if n_base_direct is None:
                n_base_direct = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        elif mode == 'fixed':
            if len(self.fixed_normal) == 3:
                n_cam = np.array(self.fixed_normal, dtype=np.float64)
            else:
                n_cam = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            if mask_for_axes is not None:
                n_cam = self._estimate_normal_pca(mask_for_axes, depth, u, v)

        if n_base_direct is None and mode not in ('camera_side', 'local_plane'):
            if n_cam is None:
                n_cam = np.array([0.0, 0.0, 1.0], dtype=np.float64)

            n_cam = self._normalize(n_cam)
            if n_cam is None:
                n_cam = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        # NOTE: Do not flip the fixed normal based on camera Z here.
        # The MoveIt node expects normal expressed in base_frame (and uses -normal as approach).

        if not self._logged_axes:
            try:
                tf_cam_to_base = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rclpy.time.Time())
                q = tf_cam_to_base.transform.rotation
                tx = tf_cam_to_base.transform.translation.x
                ty = tf_cam_to_base.transform.translation.y
                tz = tf_cam_to_base.transform.translation.z

                # Rotate camera unit axes into base frame using quaternion.
                def rot(vx: float, vy: float, vz: float):
                    qx, qy, qz, qw = q.x, q.y, q.z, q.w
                    t0 = 2.0 * (qy * vz - qz * vy)
                    t1 = 2.0 * (qz * vx - qx * vz)
                    t2 = 2.0 * (qx * vy - qy * vx)
                    rx = vx + qw * t0 + (qy * t2 - qz * t1)
                    ry = vy + qw * t1 + (qz * t0 - qx * t2)
                    rz = vz + qw * t2 + (qx * t1 - qy * t0)
                    return rx, ry, rz

                ex = rot(1.0, 0.0, 0.0)
                ey = rot(0.0, 1.0, 0.0)
                ez = rot(0.0, 0.0, 1.0)
                self.get_logger().info(
                    f"TF {camera_frame} -> {self.base_frame}: t=({tx:.3f},{ty:.3f},{tz:.3f}) "
                    f"cam_x_in_base=({ex[0]:.3f},{ex[1]:.3f},{ex[2]:.3f}) "
                    f"cam_y_in_base=({ey[0]:.3f},{ey[1]:.3f},{ey[2]:.3f}) "
                    f"cam_z_in_base=({ez[0]:.3f},{ez[1]:.3f},{ez[2]:.3f})"
                )
                self._logged_axes = True
            except TransformException:
                pass

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, p_cam.header.frame_id, rclpy.time.Time())
            p_out = do_transform_point(p_cam, tf)
        except TransformException as e:
            self.get_logger().warn(f'tf transform failed ({p_cam.header.frame_id} -> {self.base_frame}): {e}')
            return None

        # Transform direction vectors (normal, short-axis) into base_frame using tf rotation
        q = tf.transform.rotation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)

        def rot_vec(v: np.ndarray) -> np.ndarray:
            vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
            t0 = 2.0 * (qy * vz - qz * vy)
            t1 = 2.0 * (qz * vx - qx * vz)
            t2 = 2.0 * (qx * vy - qy * vx)
            rx = vx + qw * t0 + (qy * t2 - qz * t1)
            ry = vy + qw * t1 + (qz * t0 - qx * t2)
            rz = vz + qw * t2 + (qx * t1 - qy * t0)
            return np.array([rx, ry, rz], dtype=np.float64)

        if n_base_direct is not None:
            n_base = n_base_direct
        else:
            if mode == 'local_plane' and mask_for_axes is not None:
                try:
                    tf_cam_to_base = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rclpy.time.Time())
                except TransformException:
                    tf_cam_to_base = None

                if tf_cam_to_base is not None:
                    n_lp = self._estimate_local_plane_normal_base(depth, mask_for_axes, u, v, float(z), tf_cam_to_base)
                    if n_lp is not None:
                        n_base = n_lp
                    else:
                        n_base = None
                else:
                    n_base = None

            else:
                n_base = None

            if n_base is None:
                if n_cam is None:
                    n_cam = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                n_base = rot_vec(n_cam)
                n_base_n = self._normalize(n_base)
                if n_base_n is None:
                    n_base_n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                n_base = n_base_n

        if axis_cam is None:
            axis_cam = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        sa_base = rot_vec(axis_cam)
        sa_base_n = self._normalize(sa_base)
        if sa_base_n is None:
            sa_base_n = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        sa_base = sa_base_n

        # For top-down grasping of objects on a table, project the long axis onto the horizontal plane (XY plane)
        # This ensures the gripper aligns horizontally even if the detected axis has a Z component due to noise
        if mode == 'fixed_base':
            sa_base[2] = 0.0  # Zero out Z component
            sa_base_n = self._normalize(sa_base)
            if sa_base_n is not None:
                sa_base = sa_base_n
            else:
                sa_base = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        if mode == 'camera_side':
            # Camera-side normal: perpendicular to the object's long axis, biased toward the view direction.
            # a = long-axis in base_frame (sa_base), v = view direction from object to camera in base_frame.
            try:
                tf_cam = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rclpy.time.Time())
                cam_pos = np.array([
                    float(tf_cam.transform.translation.x),
                    float(tf_cam.transform.translation.y),
                    float(tf_cam.transform.translation.z),
                ], dtype=np.float64)
            except TransformException:
                cam_pos = None

            obj_pos = np.array([float(p_out.point.x), float(p_out.point.y), float(p_out.point.z)], dtype=np.float64)
            a = sa_base
            vdir = None
            if cam_pos is not None:
                vdir = self._normalize(cam_pos - obj_pos)

            if vdir is not None and self._normalize(a) is not None:
                n = vdir - float(np.dot(vdir, a)) * a
                n = self._normalize(n)
                if n is not None:
                    if float(np.dot(n, vdir)) < 0.0:
                        n = -n
                    n_base = n
        elif mode == 'local_plane':
            # local_plane already produced n_base.
            pass

        # Try to include label if available (depends on model outputs)
        label = 'unknown'
        try:
            if getattr(r0, 'names', None) is not None and r0.boxes is not None and getattr(r0.boxes, 'cls', None) is not None:
                cls_id = int(r0.boxes.cls[best_i].detach().cpu().numpy().item())
                label = str(r0.names.get(cls_id, cls_id))
        except Exception:
            pass

        p_out.header.frame_id = self.base_frame

        if self.save_debug_images and not self._saved_debug:
            try:
                os.makedirs(self.debug_image_dir, exist_ok=True)

                # Save annotated RGB
                if cv2 is not None and self._last_rgb is not None:
                    try:
                        ann = cv_rgb.copy()

                        # Overlay segmentation mask (if available)
                        try:
                            if getattr(r0, 'masks', None) is not None and r0.masks is not None and r0.masks.data is not None:
                                m = r0.masks.data[best_i].detach().cpu().numpy()
                                if m is not None:
                                    mask = m > 0.5
                                    if mask.shape[0] != ann.shape[0] or mask.shape[1] != ann.shape[1]:
                                        mask = cv2.resize(mask.astype(np.uint8), (ann.shape[1], ann.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
                                    color = np.zeros_like(ann, dtype=np.uint8)
                                    color[:, :] = (255, 0, 0)
                                    alpha = 0.35
                                    ann = np.where(mask[..., None], (ann * (1.0 - alpha) + color * alpha).astype(np.uint8), ann)
                        except Exception:
                            pass

                        pt1 = (int(round(x1)), int(round(y1)))
                        pt2 = (int(round(x2)), int(round(y2)))
                        cv2.rectangle(ann, pt1, pt2, (0, 255, 0), 2)
                        cv2.drawMarker(ann, (u, v), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
                        cv2.putText(
                            ann,
                            f"{label} {best_conf:.3f}",
                            (pt1[0], max(0, pt1[1] - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )
                        img_path = os.path.join(self.debug_image_dir, 'last_detection_annotated.png')
                        cv2.imwrite(img_path, ann)
                        self.get_logger().info(f"Saved annotated RGB to {img_path}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to save annotated RGB: {e}")

                out_path = os.path.join(self.debug_image_dir, 'last_detection.txt')
                with open(out_path, 'w') as f:
                    f.write(f"label={label}\n")
                    f.write(f"conf={best_conf}\n")
                    f.write(f"uv=({u},{v})\n")
                    f.write(f"bbox=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f})\n")
                    f.write(f"camera_frame={p_cam.header.frame_id}\n")
                    f.write(f"base_frame={self.base_frame}\n")
                    f.write(f"z_single={z_single}\n")
                    f.write(f"z_med={z}\n")
                    f.write(f"p_cam=({p_cam.point.x},{p_cam.point.y},{p_cam.point.z})\n")
                    f.write(f"p_base=({p_out.point.x},{p_out.point.y},{p_out.point.z})\n")
                    f.write(f"axis_mode={self.axis_mode}\n")
                    f.write(f"axis_img=({float(axis_img[0]):.6f},{float(axis_img[1]):.6f})\n")
                    f.write(f"axis_cam=({float(axis_cam[0]):.6f},{float(axis_cam[1]):.6f},{float(axis_cam[2]):.6f})\n")
                    f.write(f"axis_base=({float(sa_base[0]):.6f},{float(sa_base[1]):.6f},{float(sa_base[2]):.6f})\n")
                    f.write(f"normal_mode={self.normal_mode}\n")
                    if n_cam is not None:
                        f.write(f"normal_cam=({float(n_cam[0]):.6f},{float(n_cam[1]):.6f},{float(n_cam[2]):.6f})\n")
                    else:
                        f.write("normal_cam=(nan,nan,nan)\n")
                    f.write(f"normal_base=({float(n_base[0]):.6f},{float(n_base[1]):.6f},{float(n_base[2]):.6f})\n")
                self.get_logger().info(f"Saved debug text to {out_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to save debug info: {e}")
            self._saved_debug = True

        # 构造返回结果（不再发送 action）
        p_out.header.frame_id = self.base_frame

        n_msg = Vector3Stamped()
        n_msg.header = p_out.header
        n_msg.header.frame_id = self.base_frame
        n_msg.vector.x = float(n_base[0])
        n_msg.vector.y = float(n_base[1])
        n_msg.vector.z = float(n_base[2])

        sa_msg = Vector3Stamped()
        sa_msg.header = p_out.header
        sa_msg.header.frame_id = self.base_frame
        sa_msg.vector.x = float(sa_base[0])
        sa_msg.vector.y = float(sa_base[1])
        sa_msg.vector.z = float(sa_base[2])

        # 返回检测结果
        return {
            'label': label,
            'position': p_out,
            'normal': n_msg,
            'long_axis': sa_msg,
            'confidence': best_conf
        }


def main(args=None):
    rclpy.init(args=args)
    node = YoloEGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
