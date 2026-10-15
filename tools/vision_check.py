#!/usr/bin/env python3
"""GPU smoke checks; a pass does not certify live RGB-D localization or grasping."""
import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-path', default='yoloe-26l-seg.pt')
    parser.add_argument('--package-share', type=Path, help='Override ament share for a source-only check')
    parser.add_argument('--cache-dir', type=Path, help='Private writable MobileCLIP cache')
    parser.add_argument('--report', type=Path, default=Path(__file__).resolve().parents[1] / 'reports/vision-check.json')
    parser.add_argument('--image', type=Path, help='Optional real RGB image; default is a synthetic smoke input')
    args = parser.parse_args(argv)
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    report = {
        'status': 'failed', 'scope': 'GPU and Python dependency smoke check; not live-camera acceptance',
        'python': sys.executable, 'python_version': sys.version,
        'vision_python_expected': os.environ.get('VISION_PYTHON'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
        'input': str(args.image) if args.image else 'synthetic black BGR image',
        'checks': {}, 'errors': [],
    }
    modules = {}

    def check(name, function, dependencies=()):
        missing = [item for item in dependencies if report['checks'].get(item, {}).get('status') != 'passed']
        if missing:
            report['checks'][name] = {'status': 'skipped', 'reason': 'Prerequisites failed: ' + ', '.join(missing)}
            return
        start = time.monotonic()
        try:
            details = function()
            report['checks'][name] = {'status': 'passed', 'seconds': round(time.monotonic()-start, 3), 'details': details}
        except Exception as exc:
            message = f'{type(exc).__name__}: {exc}'
            report['checks'][name] = {'status': 'failed', 'seconds': round(time.monotonic()-start, 3), 'error': message}
            report['errors'].append(f'{name}: {message}')

    def load_module(name):
        module = importlib.import_module(name)
        modules[name] = module
        return {'version': getattr(module, '__version__', None), 'file': getattr(module, '__file__', None)}

    def interpreter():
        expected = os.environ.get('VISION_PYTHON')
        if expected and Path(sys.executable).absolute() != Path(expected).absolute():
            raise RuntimeError(f'Use vision interpreter {expected}, got {sys.executable}')
        return sys.executable

    check('interpreter', interpreter)
    for name in ('numpy', 'torch', 'torchvision', 'cv2', 'cv_bridge', 'sensor_msgs.msg', 'ultralytics'):
        check('import_' + name, lambda name=name: load_module(name))

    def cuda_kernels():
        torch = modules['torch']
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is unavailable in this Python environment')
        matrix = torch.ones((64, 64), device='cuda:0')
        product = matrix @ matrix
        torch.cuda.synchronize()
        if not torch.equal(product, torch.full_like(product, 64.)):
            raise RuntimeError('CUDA matrix multiply returned incorrect values')
        return {'gpu': torch.cuda.get_device_name(0), 'capability': list(torch.cuda.get_device_capability(0)),
                'cuda': torch.version.cuda, 'torch_arch_list': torch.cuda.get_arch_list()}

    def cuda_nms():
        torch, torchvision = modules['torch'], modules['torchvision']
        boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 9., 9.], [20., 20., 30., 30.]], device='cuda:0')
        scores = torch.tensor([.9, .8, .7], device='cuda:0')
        kept = torchvision.ops.nms(boxes, scores, .5)
        torch.cuda.synchronize()
        if kept.cpu().tolist() != [0, 2]:
            raise RuntimeError(f'Unexpected CUDA NMS indices: {kept.cpu().tolist()}')
        return {'indices': [0, 2], 'device': str(kept.device)}

    def bridge_roundtrip():
        np = modules['numpy']
        bridge = modules['cv_bridge'].CvBridge()
        rgb = np.arange(16*16*3, dtype=np.uint8).reshape(16, 16, 3)
        encoded = bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        bgr = bridge.imgmsg_to_cv2(encoded, desired_encoding='bgr8')
        if not np.array_equal(bgr, rgb[:, :, ::-1]):
            raise RuntimeError('cv_bridge RGB to BGR conversion differs')
        for encoding, sample in (
            ('16UC1', np.arange(256, dtype=np.uint16).reshape(16, 16)),
            ('32FC1', np.arange(256, dtype=np.float32).reshape(16, 16) / 100.),
        ):
            message = bridge.cv2_to_imgmsg(sample, encoding=encoding)
            decoded = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            if decoded.dtype != sample.dtype or not np.array_equal(decoded, sample):
                raise RuntimeError(f'cv_bridge {encoding} depth roundtrip differs')
        return {'rgb_to_bgr': True, 'depth_encodings': ['16UC1', '32FC1'], 'numpy': np.__version__}

    def yoloe_inference():
        from ament_index_python.packages import get_package_share_directory
        from yoloe_grasp.model_runtime import (
            default_cache_dir, prepare_text_cache, require_clip_tokenizer,
            resolve_model_path, text_model_working_directory,
        )
        np, torch = modules['numpy'], modules['torch']
        YOLOE = modules['ultralytics'].YOLOE
        share = args.package_share or Path(get_package_share_directory('yoloe_grasp'))
        checkpoint = resolve_model_path(args.model_path, share)
        cache = prepare_text_cache(share, args.cache_dir or default_cache_dir())
        require_clip_tokenizer()
        if args.image:
            sample = modules['cv2'].imread(str(args.image))
            if sample is None:
                raise ValueError(f'Cannot read RGB image: {args.image}')
        else:
            sample = np.zeros((640, 640, 3), dtype=np.uint8)
        with text_model_working_directory(cache):
            model = YOLOE(str(checkpoint))
            model.to('cuda:0')
            model.set_classes(['pencil', 'pen'])
            results = model.predict(sample, device='cuda:0', retina_masks=True, verbose=False)
        torch.cuda.synchronize()
        if not results or tuple(results[0].orig_shape) != sample.shape[:2]:
            raise RuntimeError('YOLOE returned no result or an unexpected input shape')
        if not next(model.model.parameters()).is_cuda:
            raise RuntimeError('YOLOE parameters are not on CUDA')
        if results[0].boxes is None or not results[0].boxes.data.is_cuda:
            raise RuntimeError('YOLOE result boxes are missing or not on CUDA')
        return {'checkpoint': str(checkpoint), 'cache': str(cache), 'device': str(next(model.model.parameters()).device),
                'input_shape': list(sample.shape), 'detections': len(results[0].boxes),
                'classes': results[0].names, 'synthetic_input': args.image is None}

    check('cuda_kernels', cuda_kernels, ('interpreter', 'import_torch'))
    check('cuda_nms', cuda_nms, ('cuda_kernels', 'import_torchvision'))
    check('cv_bridge_roundtrip', bridge_roundtrip, ('import_numpy', 'import_cv_bridge', 'import_sensor_msgs.msg'))
    check('yoloe_inference', yoloe_inference, ('interpreter', 'cuda_kernels', 'cuda_nms',
                                              'cv_bridge_roundtrip', 'import_ultralytics', 'import_cv2'))
    if not report['errors']:
        report['status'] = 'passed'
    try:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    except OSError as exc:
        report['status'] = 'failed'
        report['errors'].append(f'report_write: {exc}')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
