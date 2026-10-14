#!/usr/bin/env python3
"""Extract a ZIP, recovering CRC-verified local entries if its index is missing."""
import argparse
import json
from pathlib import Path, PurePosixPath
import shutil
import struct
import tempfile
import zipfile
import zlib


def destination(root, name):
    name = name.replace('\\', '/')
    rel = PurePosixPath(name)
    if rel.is_absolute() or '..' in rel.parts or not rel.parts or ':' in rel.parts[0]:
        raise ValueError(f'Unsafe archive path: {name!r}')
    target = (root / str(rel)).resolve()
    if not target.is_relative_to(root.resolve()):
        raise ValueError(f'Archive path escapes output directory: {name!r}')
    return target


def commit(temp, target, expected_crc, expected_size):
    temp.chmod(0o644)
    if target.exists():
        crc = 0
        with target.open('rb') as f:
            while chunk := f.read(1024 * 1024):
                crc = zlib.crc32(chunk, crc)
        if target.stat().st_size != expected_size or crc & 0xffffffff != expected_crc:
            raise FileExistsError(f'Will not overwrite different existing asset: {target}')
        temp.unlink()
        target.chmod(0o644)
    else:
        temp.replace(target)


def recover(archive, root, report):
    """Walk real DEFLATE end markers and validate data descriptors, not guessed headers."""
    with archive.open('rb') as stream:
        while True:
            offset = stream.tell()
            header = stream.read(30)
            if not header:
                break
            if len(header) < 30 or header[:4] != b'PK\x03\x04':
                report['errors'].append(f'No complete local header at byte {offset}')
                break
            _, version, flags, method, _, _, crc, compressed, size, nlen, elen = struct.unpack('<4s5H3I2H', header)
            raw_name, extra = stream.read(nlen), stream.read(elen)
            name = raw_name.decode('utf-8' if flags & 0x800 else 'cp437')
            target = destination(root, name)
            zip64 = compressed == 0xffffffff or size == 0xffffffff
            if zip64:
                cursor = 0
                while cursor + 4 <= len(extra):
                    kind, length = struct.unpack('<HH', extra[cursor:cursor+4])
                    body = extra[cursor+4:cursor+4+length]
                    cursor += length + 4
                    if kind == 1:
                        position = 0
                        if size == 0xffffffff:
                            size = struct.unpack('<Q', body[position:position+8])[0]
                            position += 8
                        if compressed == 0xffffffff:
                            compressed = struct.unpack('<Q', body[position:position+8])[0]
            if flags & 1:
                raise ValueError(f'Encrypted entry is unsupported: {name}')
            if name.endswith('/'):
                target.mkdir(parents=True, exist_ok=True)
                report['directories'] += 1
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            actual_crc, actual_size = 0, 0
            data_start = stream.tell()
            temp = None
            try:
                with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.asset-recovery-', delete=False) as output:
                    temp = Path(output.name)
                    if method == 8 and flags & 8:
                        decoder = zlib.decompressobj(-15)
                        while not decoder.eof:
                            chunk = stream.read(1024 * 1024)
                            if not chunk:
                                raise EOFError(f'Truncated DEFLATE stream: {name}')
                            unpacked = decoder.decompress(chunk)
                            output.write(unpacked)
                            actual_crc = zlib.crc32(unpacked, actual_crc)
                            actual_size += len(unpacked)
                            if actual_size > 16 * 1024**3:
                                raise ValueError(f'Entry exceeds recovery limit: {name}')
                        if decoder.unused_data:
                            stream.seek(-len(decoder.unused_data), 1)
                        actual_compressed = stream.tell() - data_start
                        first = stream.read(4)
                        if first == b'PK\x07\x08':
                            first = stream.read(4)
                        crc = struct.unpack('<I', first)[0]
                        descriptor = stream.read(16 if zip64 else 8)
                        compressed, size = struct.unpack('<QQ' if zip64 else '<II', descriptor)
                        if compressed != actual_compressed:
                            raise ValueError(f'Compressed size mismatch: {name}')
                    elif method in (0, 8) and not flags & 8:
                        decoder = zlib.decompressobj(-15) if method == 8 else None
                        remaining = compressed
                        while remaining:
                            chunk = stream.read(min(1024 * 1024, remaining))
                            if not chunk:
                                raise EOFError(f'Truncated data: {name}')
                            remaining -= len(chunk)
                            unpacked = decoder.decompress(chunk) if decoder else chunk
                            output.write(unpacked)
                            actual_crc = zlib.crc32(unpacked, actual_crc)
                            actual_size += len(unpacked)
                        if decoder and not decoder.eof:
                            raise EOFError(f'Incomplete DEFLATE data: {name}')
                    else:
                        raise ValueError(f'Unsupported method/descriptor: {method}/{flags} {name}')
                if actual_size != size or actual_crc & 0xffffffff != crc:
                    raise ValueError(f'CRC/size verification failed: {name}')
                commit(temp, target, crc, size)
                report['files'].append({'name': name, 'size': size, 'crc32': f'{crc:08x}', 'verified': True})
                if len(report['files']) % 100 == 0:
                    print(f"Recovered {len(report['files'])} verified files", flush=True)
            except (EOFError, ValueError, struct.error, zlib.error) as exc:
                if temp is not None and temp.exists():
                    temp.unlink()
                report['errors'].append(str(exc))
                report['incomplete_entry'] = name
                break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report = {'archive': str(args.archive.resolve()), 'archive_size': args.archive.stat().st_size,
              'output_root': str(args.output_root.resolve()), 'directories': 0, 'files': [], 'errors': []}
    args.output_root.mkdir(parents=True, exist_ok=True)
    try:
        with zipfile.ZipFile(args.archive) as archive:
            report['mode'] = 'normal_zip'
            for entry in archive.infolist():
                target = destination(args.output_root, entry.filename)
                if entry.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                    report['directories'] += 1
                    continue
                target.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.asset-extract-', delete=False) as out:
                    temp = Path(out.name)
                    try:
                        with archive.open(entry) as src:
                            shutil.copyfileobj(src, out, 1024 * 1024)
                    except Exception:
                        temp.unlink()
                        raise
                commit(temp, target, entry.CRC, entry.file_size)
                report['files'].append({'name': entry.filename, 'size': entry.file_size,
                                        'crc32': f'{entry.CRC:08x}', 'verified': True})
            report['status'] = 'complete'
    except zipfile.BadZipFile as exc:
        report['mode'] = 'local_entry_recovery'
        report['central_directory_error'] = str(exc)
        recover(args.archive, args.output_root, report)
        report['status'] = 'partial_recovery'
    except Exception as exc:
        report['errors'].append(str(exc))
        report['status'] = 'failed'
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'verified_files': len(report['files']),
                      'errors': report['errors']}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
