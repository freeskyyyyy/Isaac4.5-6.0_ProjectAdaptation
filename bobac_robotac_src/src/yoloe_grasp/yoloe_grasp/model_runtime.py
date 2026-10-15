"""Local checkpoint resolution shared by the node and GPU acceptance tool."""
from contextlib import contextmanager
import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import tempfile


def default_cache_dir():
    return Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'yoloe_grasp/weights'


def resolve_model_path(model_path, package_share):
    path = Path(model_path).expanduser()
    if not path.is_absolute() and path.parent == Path('.'):
        path = Path(package_share) / 'weights' / path
    path = path.resolve()
    if not path.is_file() or path.stat().st_size == 0:
        raise FileNotFoundError(f'Local YOLOE checkpoint missing or empty: {path}')
    return path


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.digest()


def prepare_text_cache(package_share, cache_dir):
    source = Path(package_share) / 'weights/mobileclip2_b.ts'
    cache = Path(cache_dir).expanduser().resolve()
    if not source.is_file() or source.stat().st_size == 0:
        raise FileNotFoundError(f'Local MobileCLIP checkpoint missing or empty: {source}')
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / source.name
    # Repair truncated or stale copies, and replace atomically for concurrent readers.
    if source.resolve() != target.resolve() and (
        not target.is_file() or source.stat().st_size != target.stat().st_size
        or _sha256(source) != _sha256(target)
    ):
        with tempfile.NamedTemporaryFile(dir=cache, prefix='.mobileclip-', delete=False) as stream:
            temporary = Path(stream.name)
        try:
            shutil.copyfile(source, temporary)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
    return cache


@contextmanager
def text_model_working_directory(cache):
    # Ultralytics v8.4.26 resolves its TorchScript text encoder relative to cwd.
    previous = Path.cwd()
    os.chdir(cache)
    try:
        yield
    finally:
        os.chdir(previous)


def require_clip_tokenizer():
    if importlib.util.find_spec('clip') is None:
        raise RuntimeError(
            'YOLOE text prompting requires the ultralytics/CLIP tokenizer (module clip). '
            'Install and pin it in the vision image; runtime pip installation is disabled.'
        )
