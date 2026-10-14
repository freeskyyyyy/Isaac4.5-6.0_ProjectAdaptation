#!/usr/bin/env python3
"""Verify actual CUDA kernels, torchvision NMS and YOLOE inference on GB10."""
import json
from pathlib import Path
import shutil
import os


def main():
    import torch
    import torchvision
    import numpy as np
    from ultralytics import YOLOE
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    report = {'status': 'failed', 'torch': torch.__version__, 'torchvision': torchvision.__version__,
              'cuda': torch.version.cuda, 'errors': []}
    try:
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is unavailable in the vision environment')
        report['gpu'] = torch.cuda.get_device_name(0)
        report['capability'] = list(torch.cuda.get_device_capability(0))
        x = torch.randn(64, 64, device='cuda')
        _ = x @ x
        boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 9., 9.]], device='cuda')
        torchvision.ops.nms(boxes, torch.tensor([.9, .8], device='cuda'), .5)
        torch.cuda.synchronize()
        # cv_bridge must work with the numpy used by this venv.
        bridge = CvBridge()
        sample = np.zeros((16, 16, 3), dtype=np.uint8)
        encoded = bridge.cv2_to_imgmsg(sample, encoding='bgr8')
        assert bridge.imgmsg_to_cv2(encoded).shape == sample.shape
        weights = Path('/workspace/bobac_ws/install/share/yoloe_grasp/weights')
        cache = Path('/workspace/course/.runtime/vision/weights')
        cache.mkdir(parents=True, exist_ok=True)
        shutil.copy2(weights / 'mobileclip2_b.ts', cache / 'mobileclip2_b.ts')
        os.chdir(cache)
        model = YOLOE(str(weights / 'yoloe-26l-seg.pt'))
        model.set_classes(['pencil', 'pen'])
        model.predict(np.zeros((640, 640, 3), dtype=np.uint8), device='cuda:0', verbose=False)
        torch.cuda.synchronize()
        report['status'] = 'passed'
    except Exception as exc:
        report['errors'].append(str(exc))
    target = Path('/workspace/course/reports/vision-check.json')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
