import argparse
import datetime as _dt
import shutil
import os
import subprocess
import struct
import sys
import time
import zlib
from pathlib import Path


def _run(cmd, timeout=20):
    return subprocess.run(
        cmd,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def _service_types():
    result = _run(['ros2', 'service', 'list', '-t'], timeout=5)
    if result.returncode != 0:
        return {}
    services = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line or '[' not in line or ']' not in line:
            continue
        name, type_part = line.split('[', 1)
        services[name.strip()] = type_part.rstrip(']').strip()
    return services


def _try_slam_toolbox_save(prefix):
    services = _service_types()
    service_type = services.get('/slam_toolbox/save_map')
    if service_type != 'slam_toolbox/srv/SaveMap':
        return False, 'slam_toolbox save_map service not available'

    request = f'{{name: {{data: "{prefix}"}}}}'
    result = _run(
        ['ros2', 'service', 'call', '/slam_toolbox/save_map', service_type, request],
        timeout=20,
    )
    if result.returncode == 0:
        return True, result.stdout.strip()
    return False, result.stderr.strip() or result.stdout.strip()


def _try_map_saver(prefix):
    result = _run([
        'ros2',
        'run',
        'nav2_map_server',
        'map_saver_cli',
        '-f',
        prefix,
        '--ros-args',
        '-p',
        'map_subscribe_transient_local:=true',
    ], timeout=30)
    if result.returncode == 0:
        return True, result.stdout.strip()
    return False, result.stderr.strip() or result.stdout.strip()


def _safe_name(value):
    cleaned = ''.join(ch if ch.isalnum() or ch in ('_', '-') else '_' for ch in value.strip())
    return cleaned.strip('_')


def _files_exist(prefix):
    yaml_path = Path(prefix + '.yaml')
    pgm_path = Path(prefix + '.pgm')
    return yaml_path.exists() and pgm_path.exists()


def _wait_for_files(prefix, timeout=5.0):
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        if _files_exist(prefix):
            return True
        time.sleep(0.1)
    return _files_exist(prefix)


def _read_pgm(path):
    data = Path(path).read_bytes()
    index = 0

    def read_token():
        nonlocal index
        while index < len(data):
            if data[index:index + 1] == b'#':
                end = data.find(b'\n', index)
                index = len(data) if end < 0 else end + 1
                continue
            if data[index:index + 1].isspace():
                index += 1
                continue
            break
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        return data[start:index]

    magic = read_token()
    if magic != b'P5':
        raise ValueError(f'unsupported PGM magic: {magic!r}')
    width = int(read_token())
    height = int(read_token())
    max_value = int(read_token())
    while index < len(data) and data[index:index + 1].isspace():
        index += 1
    pixels = data[index:index + width * height]
    if len(pixels) != width * height:
        raise ValueError('PGM pixel data is incomplete')
    if max_value != 255:
        pixels = bytes(int(value * 255 / max_value) for value in pixels)
    return width, height, pixels


def _png_chunk(name, payload):
    return (
        struct.pack('>I', len(payload))
        + name
        + payload
        + struct.pack('>I', zlib.crc32(name + payload) & 0xFFFFFFFF)
    )


def _write_png_preview(prefix):
    pgm_path = Path(prefix + '.pgm')
    png_path = Path(prefix + '.png')
    width, height, pixels = _read_pgm(pgm_path)
    rows = bytearray()
    for row in range(height):
        rows.append(0)
        start = row * width
        rows.extend(pixels[start:start + width])
    payload = (
        b'\x89PNG\r\n\x1a\n'
        + _png_chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
        + _png_chunk(b'IDAT', zlib.compress(bytes(rows), 9))
        + _png_chunk(b'IEND', b'')
    )
    png_path.write_bytes(payload)
    return png_path


def _copy_current_alias(prefix):
    source_prefix = Path(prefix)
    current_prefix = source_prefix.parent / 'current'
    copied = []
    for suffix in ('.yaml', '.pgm', '.png'):
        src = Path(str(source_prefix) + suffix)
        if not src.exists():
            continue
        dst = Path(str(current_prefix) + suffix)
        shutil.copy2(src, dst)
        copied.append(dst)
    return copied


def _parse_args():
    parser = argparse.ArgumentParser(description='Save ROS occupancy map to ~/roscar_maps.')
    parser.add_argument('--name', default='', help='Map file stem, for example map_20260626_071530.')
    parser.add_argument('--prefix', default='', help='Full output prefix without .yaml/.pgm.')
    parser.add_argument(
        '--update-current',
        action='store_true',
        help='Explicitly promote the saved map to current.{yaml,pgm,png}.',
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    map_dir = Path(os.path.expanduser('~/roscar_maps'))
    map_dir.mkdir(parents=True, exist_ok=True)
    if args.prefix:
        prefix = str(Path(os.path.expanduser(args.prefix)))
    else:
        map_name = _safe_name(args.name)
        if not map_name:
            timestamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
            map_name = f'map_n300pro_test_{timestamp}'
        prefix = str(map_dir / map_name)

    print(f'Saving map to prefix: {prefix}', flush=True)

    ok, detail = _try_slam_toolbox_save(prefix)
    if ok:
        _wait_for_files(prefix, timeout=5.0)
    if ok and _files_exist(prefix):
        print('Saved map with slam_toolbox save_map service.', flush=True)
    else:
        if ok:
            print('slam_toolbox save_map returned success but did not create both .yaml and .pgm.', flush=True)
        else:
            print(f'slam_toolbox save_map unavailable/failed: {detail}', flush=True)
        ok, detail = _try_map_saver(prefix)
        if ok:
            print('Saved map with nav2_map_server map_saver_cli.', flush=True)
        elif _wait_for_files(prefix, timeout=2.0):
            print('map_saver_cli reported failure, but map files were created; accepting saved map.', flush=True)
        else:
            print(f'map_saver_cli failed: {detail}', file=sys.stderr, flush=True)
            return 1

    yaml_path = Path(prefix + '.yaml')
    pgm_path = Path(prefix + '.pgm')
    if not yaml_path.exists() or not pgm_path.exists():
        print('map save failed: .yaml and .pgm were not both created', file=sys.stderr, flush=True)
        print(f'Expected YAML: {yaml_path}', file=sys.stderr, flush=True)
        print(f'Expected PGM: {pgm_path}', file=sys.stderr, flush=True)
        return 1
    print(f'Map YAML: {yaml_path}', flush=True)
    print(f'Map PGM: {pgm_path}', flush=True)
    try:
        png_path = _write_png_preview(prefix)
        print(f'Map PNG preview: {png_path}', flush=True)
    except Exception as exc:
        print(f'PNG preview skipped: {exc}', flush=True)
    if args.update_current:
        try:
            copied = _copy_current_alias(prefix)
            if copied:
                print('Current map alias updated explicitly:', flush=True)
                for path in copied:
                    print(f'  {path}', flush=True)
        except Exception as exc:
            print(f'Current map alias update skipped: {exc}', flush=True)
    else:
        print(
            'Formal current.* map was preserved; use --update-current only after '
            'separate validation.',
            flush=True,
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
