"""CPU regression checks with fake model/goal objects; no camera or GPU acceptance."""
import copy
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event, Lock
from types import SimpleNamespace
import time

import numpy as np
import pytest
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import CameraInfo

from yoloe_grasp.camera_data import (
    CameraFrames, backproject, depth_in_metres, optical_to_frame,
    validate_calibration,
)
from yoloe_grasp.model_runtime import (
    prepare_text_cache, resolve_model_path, text_model_working_directory,
)
from yoloe_grasp.yoloe_grasp_node import YoloEGraspNode
import yoloe_grasp.yoloe_grasp_node as node_module


def camera_triplet(stamp=10., shape=(8, 12), encoding='32FC1'):
    bridge = CvBridge()
    rgb = bridge.cv2_to_imgmsg(np.zeros((*shape, 3), np.uint8), encoding='bgr8')
    depth = bridge.cv2_to_imgmsg(np.full(shape, 1000 if encoding == '16UC1' else 1.,
                                      np.uint16 if encoding == '16UC1' else np.float32), encoding=encoding)
    info = CameraInfo()
    info.width, info.height = shape[1], shape[0]
    info.k = [100., 0., shape[1]/2, 0., 100., shape[0]/2, 0., 0., 1.]
    for message in (rgb, depth, info):
        message.header.frame_id = 'arm_Camera'
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
    return rgb, depth, info


def queued(triplet):
    frames = CameraFrames()
    for key, message in zip(('rgb', 'depth', 'info'), triplet):
        frames.add(key, message)
    return frames


def test_pairing_uses_matching_older_frame():
    frames = queued(camera_triplet())
    frames.add('rgb', camera_triplet(10.1)[0])
    snapshot, reason = frames.snapshot(int(10.1e9), .05, 1.)
    assert not reason and snapshot[0].header.stamp.sec == 10
    assert snapshot[0].header.stamp.nanosec == 0


def test_no_clock_and_missing_streams():
    assert 'rgb' in CameraFrames().snapshot(0, .05, 1.)[1]
    assert '/clock' in queued(camera_triplet()).snapshot(0, .05, 1.)[1]


@pytest.mark.parametrize('bad_stamp', [0., 8., 11.])
def test_stale_zero_or_future_stamps_rejected(bad_stamp):
    assert queued(camera_triplet(bad_stamp)).snapshot(int(10e9), .05, 1.)[0] is None


def test_skew_and_static_camera_info():
    triplet = camera_triplet()
    triplet[2].header.stamp.sec = 0
    frames = queued(triplet)
    assert frames.snapshot(int(10e9), .05, 1.)[0] is None
    assert frames.snapshot(int(10e9), .05, 1., static_info=True)[0] is not None
    skew = camera_triplet()
    skew[1].header.stamp.nanosec = 90_000_000
    assert queued(skew).snapshot(int(10e9), .05, 1.)[0] is None


def test_reception_age_and_reset(monkeypatch):
    monkeypatch.setattr('yoloe_grasp.camera_data.time.monotonic', lambda: 1.)
    frames = queued(camera_triplet())
    monkeypatch.setattr('yoloe_grasp.camera_data.time.monotonic', lambda: 3.)
    assert frames.snapshot(int(10e9), .05, 1.)[0] is None
    frames.add('rgb', camera_triplet(1.)[0])
    assert frames.snapshot(int(1e9), .05, 1.)[0] is None
    frames.add('depth', camera_triplet(1.)[1])
    frames.add('info', camera_triplet(1.)[2])
    assert frames.snapshot(int(1e9), .05, 1.)[0] is not None


@pytest.mark.parametrize('encoding,values,expected', [
    ('16UC1', [[0, 1000, 2500]], [[np.nan, 1., 2.5]]),
    ('32FC1', [[0., -1., np.inf, np.nan, 1.5]], [[np.nan, np.nan, np.nan, np.nan, 1.5]]),
])
def test_depth_units_and_invalid_values(encoding, values, expected):
    assert np.allclose(depth_in_metres(values, encoding), expected, equal_nan=True)


def test_unsupported_depth_and_shape():
    with pytest.raises(ValueError):
        depth_in_metres([[1]], 'mono16')
    with pytest.raises(ValueError):
        depth_in_metres([1], '32FC1')


@pytest.mark.parametrize('fault', ['size', 'frame', 'empty_frame', 'fx', 'nan', 'distortion', 'binning', 'roi'])
def test_bad_calibration_rejected(fault):
    rgb, depth, info = camera_triplet()
    if fault == 'size':
        depth.width += 1
    elif fault == 'frame':
        info.header.frame_id = 'other_camera'
    elif fault == 'empty_frame':
        rgb.header.frame_id = ''
    elif fault == 'fx':
        info.k[0] = 0
    elif fault == 'nan':
        info.k[2] = np.nan
    elif fault == 'distortion':
        info.d = [0.1]
    elif fault == 'binning':
        info.binning_x = 2
    else:
        info.roi.x_offset = 3
    with pytest.raises(ValueError):
        validate_calibration(rgb, depth, info)


def test_optical_conventions_and_backprojection():
    k = validate_calibration(*camera_triplet())
    assert np.allclose(backproject(6, 4, 2., k), [0, 0, 2.])
    for convention in ('optical', 'usd', 'ros_body'):
        rotation = optical_to_frame(convention)
        assert np.allclose(rotation @ rotation.T, np.eye(3))
        assert np.isclose(np.linalg.det(rotation), 1.)
    assert np.allclose(optical_to_frame('usd') @ [1, 2, 3], [1, -2, -3])
    assert np.allclose(optical_to_frame('ros_body') @ [1, 2, 3], [3, -1, -2])
    with pytest.raises(ValueError):
        optical_to_frame('guess')


def test_paths_cache_repair_and_cwd_restore(tmp_path, monkeypatch):
    share = tmp_path / 'install/share/yoloe_grasp'
    weights = share / 'weights'
    weights.mkdir(parents=True)
    model = weights / 'yoloe-26l-seg.pt'
    model.write_bytes(b'unit-test checkpoint')
    text = weights / 'mobileclip2_b.ts'
    text.write_bytes(b'good')
    assert resolve_model_path(model.name, share) == model
    assert resolve_model_path(str(model), share) == model
    monkeypatch.chdir(tmp_path)
    assert resolve_model_path(str(model.relative_to(tmp_path)), share) == model
    with pytest.raises(FileNotFoundError):
        resolve_model_path('missing.pt', share)
    cache = prepare_text_cache(share, tmp_path / 'cache')
    (cache / text.name).write_bytes(b'bad!')  # Same-size stale content must be repaired.
    prepare_text_cache(share, cache)
    assert (cache / text.name).read_bytes() == b'good'
    assert not list(cache.glob('.mobileclip-*'))
    before = Path.cwd()
    with pytest.raises(RuntimeError):
        with text_model_working_directory(cache):
            assert Path.cwd() == cache
            raise RuntimeError('test')
    assert Path.cwd() == before


class ArrayTensor:
    def __init__(self, values):
        self.values = np.asarray(values)
    def detach(self):
        return self
    def cpu(self):
        return self
    def numpy(self):
        return self.values
    def item(self):
        return self.values.item()
    def __getitem__(self, index):
        return ArrayTensor(self.values[index])


class Boxes:
    def __init__(self):
        self.conf = ArrayTensor([.8])
        self.xyxy = ArrayTensor([[2, 2, 10, 6]])
        self.cls = ArrayTensor([0])
    def __len__(self):
        return 1


def fake_node():
    node = object.__new__(YoloEGraspNode)
    node.bridge = CvBridge()
    node.normal_mode = 'fixed_base'
    node.fixed_normal = [0., 0., -1.]
    node.axis_mode = 'mask_long'
    node.axis_step_px = 10
    node.device = 'cuda:0'  # Only passed to the fake model; no CUDA call.
    node.save_debug_images = False
    node.base_frame = 'base_link_arm'
    node.camera_frame = ''
    node._active_classes = ['pen']
    node._optical_rotation = np.eye(3)
    node._last_diagnostic = ''
    node._throttle_last = {}
    node.get_logger = lambda: SimpleNamespace(warn=lambda text: None, error=lambda text: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(10e9)))
    return node


@pytest.mark.parametrize('convention,expected_z', [('optical', 1.), ('usd', -1.)])
@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
def test_detection_geometry_headers_and_confidence(convention, expected_z, encoding):
    node = fake_node()
    node._optical_rotation = optical_to_frame(convention)
    triplet = camera_triplet(encoding=encoding)
    mask = np.zeros((8, 12), dtype=float)
    mask[3:5, 3:10] = 1
    prediction = SimpleNamespace(boxes=Boxes(), masks=SimpleNamespace(data=ArrayTensor([mask])), names={0: 'pen'})
    kwargs_seen = []
    node.model = SimpleNamespace(predict=lambda image, **kwargs: kwargs_seen.append(kwargs) or [prediction])
    tf = TransformStamped()
    tf.header.frame_id = node.base_frame
    tf.transform.rotation.w = 1.
    tf.transform.translation.x = 2.
    original_header = copy.deepcopy(triplet[0].header)
    detection = node._detect_once(triplet, validate_calibration(*triplet), 'arm_Camera', tf, ['pen'], .4, Event())
    assert detection['position'].point.x == pytest.approx(2.)
    assert detection['position'].point.z == pytest.approx(expected_z)
    assert abs(detection['long_axis'].vector.x) == pytest.approx(1.)
    assert detection['normal'].vector.z == -1.
    assert triplet[0].header == original_header
    assert detection['position'].header.stamp == original_header.stamp
    assert detection['position'].header.frame_id == 'base_link_arm'
    assert kwargs_seen[0]['conf'] == .4 and kwargs_seen[0]['retina_masks'] is True


def test_plane_normal_uses_optical_rotation():
    node = fake_node()
    node.local_plane_radius_px = 10
    node.local_plane_depth_tol = .01
    node.local_plane_min_points = 8
    depth = np.ones((8, 12))
    k = validate_calibration(*camera_triplet())
    for convention, sign in [('optical', -1.), ('usd', 1.)]:
        rotation = optical_to_frame(convention)
        normal = node._plane_normal(depth, np.ones(depth.shape, bool), 6, 4, 1., k, rotation,
                                    np.zeros(3), rotation @ [0, 0, 1.])
        assert normal[2] == pytest.approx(sign)


class FakeGoal:
    def __init__(self, timeout=.12, cancel_on_detect=False):
        self.request = SimpleNamespace(timeout=timeout, confidence_threshold=.4, target_classes=['pen'])
        self.is_cancel_requested = False
        self.status = None
        self.feedback = []
        self.cancel_on_detect = cancel_on_detect
    def publish_feedback(self, feedback):
        self.feedback.append(copy.deepcopy(feedback))
        if self.cancel_on_detect and feedback.state == 'DETECTING':
            self.is_cancel_requested = True
    def canceled(self):
        self.status = 'canceled'
    def abort(self):
        self.status = 'aborted'
    def succeed(self):
        self.status = 'succeeded'


def action_node(monkeypatch, snapshot=None):
    monkeypatch.setattr(node_module.rclpy, 'ok', lambda: True)
    node = fake_node()
    node._goal_lock = Lock()
    node._goal_reserved = True
    node._inference_future = None
    node.classes = ['pen']
    node.conf = .01
    node.sync_tolerance_sec = .05
    node.max_data_age_sec = 1.
    node.allow_static_camera_info = False
    node._frames = SimpleNamespace(snapshot=lambda *args: (snapshot, 'RGB/depth unavailable'))
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: tf)
    return node


def test_wall_timeout_with_frozen_sim_clock(monkeypatch):
    node = action_node(monkeypatch)
    goal = FakeGoal()
    start = time.monotonic()
    result = node.execute_callback(goal)
    assert goal.status == 'aborted' and not result.success
    assert 'RGB/depth unavailable' in result.message
    assert .1 <= time.monotonic() - start < .6
    assert not node._goal_reserved


@pytest.mark.parametrize('cancel', [False, True])
def test_timeout_and_cancel_while_inference_busy(monkeypatch, cancel):
    node = action_node(monkeypatch, camera_triplet())
    release = Event()
    node._worker = ThreadPoolExecutor(max_workers=1)
    node._detect_once = lambda *args: release.wait(1.) and None
    goal = FakeGoal(cancel_on_detect=cancel)
    try:
        result = node.execute_callback(goal)
        assert not result.success
        assert goal.status == ('canceled' if cancel else 'aborted')
        assert node._inference_future is not None and not node._inference_future.done()
        assert node.goal_callback(goal.request) == node_module.GoalResponse.REJECT
    finally:
        release.set()
        node._worker.shutdown()
    assert node.goal_callback(goal.request) == node_module.GoalResponse.ACCEPT


def test_action_success_and_tf_acquisition_stamp(monkeypatch):
    triplet = camera_triplet()
    node = action_node(monkeypatch, triplet)
    node._worker = ThreadPoolExecutor(max_workers=1)
    point = PointStamped()
    point.header = copy.deepcopy(triplet[0].header)
    captured_tf = []
    node.tf_buffer.lookup_transform = lambda *args: captured_tf.append(args) or TransformStamped()
    detection = dict(position=point, label='pen', confidence=.8,
                     normal=node_module.Vector3Stamped(), long_axis=node_module.Vector3Stamped())
    node._detect_once = lambda *args: detection
    try:
        goal = FakeGoal(timeout=.5)
        result = node.execute_callback(goal)
        assert result.success and goal.status == 'succeeded'
        assert captured_tf[0][0:2] == ('base_link_arm', 'arm_Camera')
        assert captured_tf[0][2].nanoseconds == int(10e9)
    finally:
        node._worker.shutdown()


def test_inference_exception_returns_failure(monkeypatch):
    node = action_node(monkeypatch, camera_triplet())
    node._worker = ThreadPoolExecutor(max_workers=1)
    def failed(*args):
        raise RuntimeError('operator ABI mismatch')
    node._detect_once = failed
    try:
        goal = FakeGoal(timeout=.5)
        result = node.execute_callback(goal)
        assert not result.success and goal.status == 'aborted'
        assert 'operator ABI mismatch' in result.message
    finally:
        node._worker.shutdown()


@pytest.mark.parametrize('field,value', [('timeout', -1.), ('timeout', float('nan')),
                                         ('confidence_threshold', 1.1), ('target_classes', [' '])])
def test_bad_goal_rejected(field, value):
    node = fake_node()
    request = FakeGoal().request
    setattr(request, field, value)
    assert node.goal_callback(request) == node_module.GoalResponse.REJECT


def test_completed_inference_rejects_wall_stale_sample(monkeypatch):
    node = action_node(monkeypatch, camera_triplet())
    node.max_data_age_sec = .02
    node._worker = ThreadPoolExecutor(max_workers=1)
    point = PointStamped()
    point.header = copy.deepcopy(camera_triplet()[0].header)
    detection = dict(position=point, label='pen', confidence=.8,
                     normal=node_module.Vector3Stamped(), long_axis=node_module.Vector3Stamped())
    node._detect_once = lambda *args: detection
    try:
        goal = FakeGoal(timeout=.15)
        result = node.execute_callback(goal)
        assert not result.success and goal.status == 'aborted'
        assert 'became stale' in result.message
    finally:
        node._worker.shutdown()


def test_timeout_preserves_detection_failure(monkeypatch):
    node = action_node(monkeypatch, camera_triplet())
    node._worker = ThreadPoolExecutor(max_workers=1)
    def no_depth(*args):
        node._diagnose('Detected mask contains no valid metric depth')
        return None
    node._detect_once = no_depth
    try:
        goal = FakeGoal(timeout=.15)
        result = node.execute_callback(goal)
        assert goal.status == 'aborted'
        assert 'no valid metric depth' in result.message
    finally:
        node._worker.shutdown()


@pytest.mark.parametrize('reliability,expected', [('reliable', 'RELIABLE'), ('best_effort', 'BEST_EFFORT')])
def test_camera_qos_selection_preserves_queue_and_other_subscribers(reliability, expected):
    from rclpy.qos import qos_profile_sensor_data
    original = copy.copy(qos_profile_sensor_data)
    profile = node_module.camera_subscription_qos(reliability)
    assert profile.reliability.name == expected
    assert profile.depth == original.depth
    assert profile.history == original.history
    assert profile.durability == original.durability
    assert qos_profile_sensor_data == original
    # Per-stream profiles must not share mutable QoS state.
    profile.depth += 1
    assert node_module.camera_subscription_qos(reliability).depth == original.depth


@pytest.mark.parametrize('reliability', ['reliabel', 'system_default'])
def test_camera_qos_rejects_unsupported_reliability(reliability):
    with pytest.raises(ValueError, match='reliable.*best_effort'):
        node_module.camera_subscription_qos(reliability)
