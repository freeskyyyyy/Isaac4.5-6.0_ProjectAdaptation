"""YOLOE RGB-D localization action with timestamped TF and bounded requests."""
import copy
from concurrent.futures import ThreadPoolExecutor
import math
import os
from pathlib import Path
import sys
from threading import Event, Lock
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from eco65b_grasp_interfaces.action import DetectObject

from .camera_data import (
    CameraFrames, backproject, depth_in_metres, optical_to_frame, stamp_ns,
    validate_calibration,
)
from .model_runtime import (
    default_cache_dir, prepare_text_cache, require_clip_tokenizer,
    resolve_model_path, text_model_working_directory,
)


def camera_subscription_qos(reliability):
    """Select reliability without changing the sensor queue or global QoS profile."""
    policies = {'reliable': ReliabilityPolicy.RELIABLE,
                'best_effort': ReliabilityPolicy.BEST_EFFORT}
    if reliability not in policies:
        raise ValueError(f'Invalid camera QoS reliability {reliability!r}; '
                         'expected reliable or best_effort')
    profile = copy.copy(qos_profile_sensor_data)
    profile.reliability = policies[reliability]
    return profile


class YoloEGraspNode(Node):
    def __init__(self):
        super().__init__('yoloe_grasp_node')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        defaults = {
            'model_path': 'yoloe-26l-seg.pt', 'classes': ['pencil', 'pen'],
            'conf': 0.01, 'device': 'cuda:0', 'save_debug_images': True,
            'debug_image_dir': str(default_cache_dir().parent / 'debug'),
            'text_model_cache_dir': str(default_cache_dir()),
            'rgb_topic': '/arm_camera/rgb', 'depth_topic': '/arm_camera/depth',
            'camera_info_topic': '/arm_camera/camera_info', 'base_frame': 'base_link_arm',
            'rgb_qos_reliability': 'best_effort', 'depth_qos_reliability': 'best_effort',
            'camera_info_qos_reliability': 'best_effort',
            'camera_frame': '', 'camera_frame_convention': 'optical',
            # Kept for old parameter files; explicit convention replaces guessing.
            'disable_optical_compensation': True,
            'sync_tolerance_sec': 0.05, 'max_data_age_sec': 1.0,
            'allow_static_camera_info': False,
            'normal_mode': 'local_plane', 'fixed_normal': [0., 0., 1.],
            'axis_mode': 'mask_long', 'axis_step_px': 10,
            'local_plane_radius_px': 12, 'local_plane_depth_tol': 0.015,
            'local_plane_min_points': 80,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)
        if not self.disable_optical_compensation:
            raise ValueError('Legacy optical compensation is ambiguous; set '
                             'disable_optical_compensation=true and camera_frame_convention '
                             'to optical, usd, or ros_body explicitly')
        self._optical_rotation = optical_to_frame(self.camera_frame_convention)
        if not (math.isfinite(self.sync_tolerance_sec) and self.sync_tolerance_sec >= 0
                and math.isfinite(self.max_data_age_sec) and self.max_data_age_sec > 0):
            raise ValueError('Invalid sync_tolerance_sec or max_data_age_sec')
        if not math.isfinite(self.conf) or not 0 < self.conf <= 1:
            raise ValueError('conf must be in (0, 1]')
        if self.normal_mode not in ('local_plane', 'camera_side', 'fixed_base', 'fixed', 'pca'):
            raise ValueError(f'Unsupported normal_mode: {self.normal_mode}')
        if self.axis_mode not in ('mask_long', 'mask_short'):
            raise ValueError(f'Unsupported axis_mode: {self.axis_mode}')
        if self.normal_mode in ('fixed', 'fixed_base') and (
            len(self.fixed_normal) != 3 or self._normalize(np.asarray(self.fixed_normal)) is None
        ):
            raise ValueError('fixed_normal must contain three finite, nonzero components')
        rgb_qos = camera_subscription_qos(self.rgb_qos_reliability)
        depth_qos = camera_subscription_qos(self.depth_qos_reliability)
        info_qos = camera_subscription_qos(self.camera_info_qos_reliability)
        self.debug_image_dir = str(Path(self.debug_image_dir).expanduser().resolve())
        self._frames = CameraFrames()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, self.rgb_topic, lambda m: self._frames.add('rgb', m), rgb_qos)
        self.create_subscription(Image, self.depth_topic, lambda m: self._frames.add('depth', m), depth_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, lambda m: self._frames.add('info', m), info_qos)
        self._goal_lock = Lock()
        self._goal_reserved = False
        self._inference_future = None
        self._last_diagnostic = 'Waiting for camera data'
        self._throttle_last = {}
        self._active_classes = None
        self._saved_debug = False
        self.pkg_share = get_package_share_directory('yoloe_grasp')
        self.model_path = resolve_model_path(self.model_path, self.pkg_share)
        self.text_model_cache_dir = prepare_text_cache(self.pkg_share, self.text_model_cache_dir)
        self.model = self._load_model()
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yoloe-inference')
        self._action_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, DetectObject, '/detect_object', execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )
        self.get_logger().info(
            f'/detect_object ready; model={self.model_path}, device={self.device}, '
            f'python={sys.executable}, camera axes={self.camera_frame_convention}, '
            f'cache={self.text_model_cache_dir}, use_sim_time=true, '
            f'camera QoS rgb={self.rgb_qos_reliability}, depth={self.depth_qos_reliability}, '
            f'camera_info={self.camera_info_qos_reliability}'
        )

    def _load_model(self):
        expected_python = os.environ.get('VISION_PYTHON')
        if expected_python and Path(sys.executable).absolute() != Path(expected_python).absolute():
            raise RuntimeError(f'Wrong vision interpreter: {sys.executable}; expected {expected_python}')
        # Do not install packages implicitly when the robot starts.
        os.environ['YOLO_AUTOINSTALL'] = 'false'
        import torch
        from ultralytics import YOLOE
        require_clip_tokenizer()
        if str(self.device).startswith('cuda') and not torch.cuda.is_available():
            raise RuntimeError(f'CUDA unavailable in vision Python {sys.executable}')
        with text_model_working_directory(self.text_model_cache_dir):
            model = YOLOE(str(self.model_path))
            model.to(self.device)
            if self.classes:
                model.set_classes(list(self.classes))
                self._active_classes = list(self.classes)
        return model

    def goal_callback(self, request):
        if (not math.isfinite(request.timeout) or request.timeout < 0
                or not math.isfinite(request.confidence_threshold)
                or not 0 <= request.confidence_threshold <= 1
                or any(not value.strip() for value in request.target_classes)):
            self.get_logger().error('Rejecting detection goal: invalid timeout, confidence, or classes')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_reserved or (self._inference_future is not None and not self._inference_future.done()):
                self.get_logger().warn('Rejecting detection goal: another goal or inference is still active')
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Detection cancellation accepted')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = DetectObject.Result()
        feedback = DetectObject.Feedback()
        feedback.state = 'WAITING_IMAGE'
        feedback.detection_attempts = 0
        stop = Event()
        timeout = goal_handle.request.timeout or 30.0
        deadline = time.monotonic() + timeout
        classes = list(goal_handle.request.target_classes or self.classes)
        confidence = goal_handle.request.confidence_threshold or self.conf
        future = None
        previous_pair = None
        sample_received_at = None
        self._last_diagnostic = 'Waiting for synchronized camera data'
        self._saved_debug = False
        goal_handle.publish_feedback(feedback)
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    stop.set()
                    result.message = 'Detection canceled; last diagnostic: ' + self._last_diagnostic
                    goal_handle.canceled()
                    return result
                if time.monotonic() >= deadline:
                    stop.set()
                    result.message = (f'Detection timed out after {timeout:.2f}s wall time; '
                                      f'attempts={feedback.detection_attempts}; last diagnostic: {self._last_diagnostic}')
                    self.get_logger().error(result.message)
                    goal_handle.abort()
                    return result
                if future is not None:
                    if not future.done():
                        time.sleep(0.02)
                        continue
                    detection = future.result()
                    future = None
                    if detection is not None:
                        age = (self.get_clock().now().nanoseconds - stamp_ns(detection['position'])) / 1e9
                        wall_age = time.monotonic() - sample_received_at
                        if (age < -self.sync_tolerance_sec or age > self.max_data_age_sec
                                or wall_age > self.max_data_age_sec):
                            self._diagnose('Result became stale or simulation reset during inference')
                        else:
                            result.success = True
                            result.message = 'Detection succeeded'
                            result.detected_class = detection['label']
                            result.object_position = detection['position']
                            result.normal = detection['normal']
                            result.long_axis = detection['long_axis']
                            result.confidence = detection['confidence']
                            feedback.state = 'FOUND'
                            goal_handle.publish_feedback(feedback)
                            goal_handle.succeed()
                            return result
                snapshot, reason = self._frames.snapshot(
                    self.get_clock().now().nanoseconds, self.sync_tolerance_sec,
                    self.max_data_age_sec, self.allow_static_camera_info,
                )
                if snapshot is None:
                    self._diagnose(reason)
                    feedback.state = 'WAITING_IMAGE'
                else:
                    pair = tuple(stamp_ns(m) for m in snapshot)
                    if pair == previous_pair:
                        # Retain the useful detection failure in the eventual timeout result.
                        feedback.state = 'WAITING_IMAGE'
                    else:
                        try:
                            k = validate_calibration(*snapshot)
                            frame = self.camera_frame or snapshot[0].header.frame_id
                            tf = self.tf_buffer.lookup_transform(
                                self.base_frame, frame, Time.from_msg(snapshot[0].header.stamp),
                            )
                        except (ValueError, TransformException) as exc:
                            self._diagnose(str(exc))
                        else:
                            previous_pair = pair
                            sample_received_at = getattr(snapshot, 'received_at', time.monotonic())
                            feedback.state = 'DETECTING'
                            feedback.detection_attempts += 1
                            self._last_diagnostic = 'YOLOE inference in progress'
                            with self._goal_lock:
                                future = self._worker.submit(self._detect_once, snapshot, k, frame, tf, classes, confidence, stop)
                                self._inference_future = future
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
            result.message = 'ROS shutdown during detection'
            goal_handle.abort()
            return result
        except Exception as exc:
            result.message = f'Detection failed: {type(exc).__name__}: {exc}; python={sys.executable}'
            self.get_logger().error(result.message)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        finally:
            stop.set()
            with self._goal_lock:
                self._goal_reserved = False

    def _diagnose(self, message):
        self._last_diagnostic = message
        now = time.monotonic()
        if now - self._throttle_last.get('last_log', -math.inf) >= 2.0:
            # Throttle even when timestamps or TF exception text changes each tick.
            self._throttle_last['last_log'] = now
            self.get_logger().warn(message)

    @staticmethod
    def _normalize(vector):
        length = float(np.linalg.norm(vector))
        if not np.isfinite(length) or length < 1e-9:
            return None
        return vector / length

    @staticmethod
    def _rotation(tf):
        q = tf.transform.rotation
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
        length = np.linalg.norm(quaternion)
        if not np.isfinite(length) or length < 1e-9:
            raise ValueError('TF contains an invalid quaternion')
        x, y, z, w = quaternion / length
        return np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ])

    def _plane_normal(self, depth, mask, u, v, z, k, rotation, camera_position, point, local=True):
        valid = mask & np.isfinite(depth)
        if local:
            yy, xx = np.indices(depth.shape)
            radius = max(2, self.local_plane_radius_px)
            valid &= ((np.abs(xx - u) <= radius) & (np.abs(yy - v) <= radius)
                      & (np.abs(depth - z) <= self.local_plane_depth_tol))
        ys, xs = np.where(valid)
        if xs.size < max(3, self.local_plane_min_points):
            return None
        # Bound work for full-mask PCA.
        stride = max(1, math.ceil(xs.size / 10000))
        xs, ys = xs[::stride], ys[::stride]
        zs = depth[ys, xs]
        points = np.column_stack(((xs-k[0, 2])*zs/k[0, 0], (ys-k[1, 2])*zs/k[1, 1], zs))
        centered = points - points.mean(axis=0)
        eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered)
        if eigenvalues[1] <= 1e-12:
            return None
        normal = self._normalize(rotation @ eigenvectors[:, 0])
        if normal is not None and np.dot(normal, camera_position - point) < 0:
            normal = -normal
        return normal

    def _detect_once(self, snapshot, k, frame, tf, classes, confidence, stop):
        if stop.is_set():
            return None
        rgb, depth_message, _ = snapshot
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        depth = depth_in_metres(self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough'), depth_message.encoding)
        if cv_rgb.shape[:2] != depth.shape or depth.shape != (rgb.height, rgb.width):
            raise ValueError('Decoded RGB/depth dimensions do not match camera headers')
        if classes != self._active_classes and classes:
            with text_model_working_directory(self.text_model_cache_dir):
                self.model.set_classes(classes)
            self._active_classes = list(classes)
        if stop.is_set():
            return None
        predictions = self.model.predict(cv_rgb, conf=confidence, device=self.device, verbose=False, retina_masks=True)
        if stop.is_set():
            return None
        if not predictions or predictions[0].boxes is None or len(predictions[0].boxes) == 0:
            self._diagnose(f'YOLOE found no target at confidence {confidence:.3f}')
            return None
        prediction = predictions[0]
        scores = prediction.boxes.conf.detach().cpu().numpy()
        best = int(np.argmax(scores))
        score = float(scores[best])
        if not math.isfinite(score) or score < confidence:
            self._diagnose('YOLOE returned an invalid/below-threshold confidence')
            return None
        bbox = prediction.boxes.xyxy[best].detach().cpu().numpy()
        if not np.isfinite(bbox).all():
            raise ValueError('YOLOE returned a nonfinite bounding box')
        u, v = int(round((bbox[0]+bbox[2])/2)), int(round((bbox[1]+bbox[3])/2))
        mask = None
        if prediction.masks is not None and prediction.masks.data is not None:
            mask = prediction.masks.data[best].detach().cpu().numpy() > 0.5
            if mask.shape != depth.shape:
                raise ValueError(f'retina_masks shape {mask.shape} != registered image {depth.shape}')
            ys, xs = np.where(mask & np.isfinite(depth))
            if xs.size == 0:
                self._diagnose('Detected mask contains no valid metric depth')
                return None
            # Use a real mask pixel near its centroid; bbox center can be background.
            center = np.array([np.median(xs), np.median(ys)])
            nearest = int(np.argmin((xs-center[0])**2 + (ys-center[1])**2))
            u, v = int(xs[nearest]), int(ys[nearest])
        if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
            self._diagnose(f'Detection center outside registered depth: ({u}, {v})')
            return None
        z = float(depth[v, u])
        if not math.isfinite(z) or z <= 0:
            self._diagnose(f'Invalid metric depth at detection center ({u}, {v})')
            return None
        point_optical = backproject(u, v, z, k)
        p_cam = PointStamped()
        p_cam.header = copy.deepcopy(rgb.header)
        p_cam.header.frame_id = frame
        point_camera = self._optical_rotation @ point_optical
        p_cam.point.x, p_cam.point.y, p_cam.point.z = map(float, point_camera)
        p_out = do_transform_point(p_cam, tf)
        p_out.header = copy.deepcopy(rgb.header)
        p_out.header.frame_id = self.base_frame
        rotation = self._rotation(tf) @ self._optical_rotation
        translation = tf.transform.translation
        camera_position = np.array([translation.x, translation.y, translation.z])
        point_base = np.array([p_out.point.x, p_out.point.y, p_out.point.z])
        if not np.isfinite(point_base).all():
            raise ValueError('Nonfinite transformed detection point')
        axis_image = np.array([1., 0.])
        if mask is not None:
            ys, xs = np.where(mask)
            if xs.size >= 3:
                pixels = np.column_stack((xs, ys)).astype(float)
                pixels -= pixels.mean(axis=0)
                _, eigenvectors = np.linalg.eigh(pixels.T @ pixels)
                axis_image = eigenvectors[:, -1 if self.axis_mode == 'mask_long' else 0]
        # Two points on the same local depth plane avoid using background depth for the axis.
        step = max(2, self.axis_step_px)
        p2_optical = backproject(u + axis_image[0]*step, v + axis_image[1]*step, z, k)
        long_axis = self._normalize(rotation @ (p2_optical - point_optical))
        if long_axis is None:
            raise ValueError('Degenerate detected axis')
        mode = self.normal_mode
        normal = None
        if mode == 'fixed_base':
            normal = self._normalize(np.array(self.fixed_normal, dtype=float))
            long_axis[2] = 0
            long_axis = self._normalize(long_axis)
        elif mode == 'fixed':
            normal = self._normalize(rotation @ np.array(self.fixed_normal, dtype=float))
        elif mode in ('local_plane', 'pca') and mask is not None:
            normal = self._plane_normal(depth, mask, u, v, z, k, rotation, camera_position, point_base, mode == 'local_plane')
        elif mode == 'camera_side':
            view = camera_position - point_base
            normal = self._normalize(view - np.dot(view, long_axis) * long_axis)
        if normal is None or long_axis is None:
            self._diagnose(f'Cannot estimate a valid normal/long axis for normal_mode={mode}')
            return None
        cls_id = int(prediction.boxes.cls[best].detach().cpu().item())
        label = str(prediction.names[cls_id])
        n_msg, axis_msg = Vector3Stamped(), Vector3Stamped()
        for message, vector in ((n_msg, normal), (axis_msg, long_axis)):
            message.header = copy.deepcopy(p_out.header)
            message.vector.x, message.vector.y, message.vector.z = map(float, vector)
        if self.save_debug_images and not self._saved_debug and not stop.is_set():
            self._save_debug(prediction, u, v, p_out, n_msg, axis_msg, depth_message.encoding)
        return {'label': label, 'position': p_out, 'normal': n_msg, 'long_axis': axis_msg, 'confidence': score}

    def _save_debug(self, prediction, u, v, point, normal, axis, encoding):
        try:
            directory = Path(self.debug_image_dir)
            directory.mkdir(parents=True, exist_ok=True)
            annotated = prediction.plot()
            cv2.drawMarker(annotated, (u, v), (0, 0, 255), markerType=cv2.MARKER_CROSS)
            if not cv2.imwrite(str(directory / 'last_detection_annotated.png'), annotated):
                raise OSError('cv2.imwrite returned false')
            (directory / 'last_detection.txt').write_text(
                f'model={self.model_path}\npython={sys.executable}\n'
                f'stamp_ns={stamp_ns(point)}\nbase_frame={point.header.frame_id}\n'
                f'camera_frame_convention={self.camera_frame_convention}\ndepth_encoding={encoding}\n'
                f'uv=({u},{v})\nposition={point.point}\nnormal={normal.vector}\nlong_axis={axis.vector}\n'
            )
            self._saved_debug = True
        except Exception as exc:
            self.get_logger().warn(f'Failed to save detection debug data: {exc}')

    def destroy_node(self):
        self._worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    os.environ.setdefault('ROS_DOMAIN_ID', '44')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    if os.environ['ROS_DOMAIN_ID'] != '44' or os.environ['RMW_IMPLEMENTATION'] != 'rmw_cyclonedds_cpp':
        raise RuntimeError('Vision requires ROS_DOMAIN_ID=44 and Cyclone DDS')
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = YoloEGraspNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
