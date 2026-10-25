"""加载显式构建的离线程序；不自动调用编译器，不授予现场或运动资格。"""
import hashlib
import json
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
ENVIRONMENT_KEY = 'I2R_KCF_REPLAY_BIN_DIR'


def configured_native(name, expected_sources=None):
    """核对构建清单、程序及源码，准备子进程 DLL 路径；只在 macOS 可回退辅助编译。"""
    directory = os.environ.get(ENVIRONMENT_KEY)
    if not directory:
        if sys.platform != 'darwin':
            raise RuntimeError('Build i2r_rlc_kcf_diagnostics in VS2022, then set '
                               + ENVIRONMENT_KEY + ' to the directory containing replay-native.json')
        return None
    folder = pathlib.Path(directory).resolve()
    manifest = folder / 'replay-native.json'
    if not manifest.is_file():
        raise RuntimeError('Missing replay-native.json in explicit native directory: ' + str(folder))
    record = json.loads(manifest.read_text(encoding='utf-8'))
    if record.get('schema_version') != 1:
        raise RuntimeError('Unsupported replay-native.json schema')
    entry = record.get('binaries', {}).get(name)
    if not isinstance(entry, dict):
        raise RuntimeError('Native replay program missing from replay-native.json: ' + name)
    filename = entry.get('file')
    if not isinstance(filename, str) or pathlib.Path(filename).name != filename:
        raise RuntimeError('Native replay filename must be a local filename: ' + name)
    binary = folder / filename
    if not binary.is_file() or hashlib.sha256(binary.read_bytes()).hexdigest() != entry.get('sha256'):
        raise RuntimeError('Native binary missing or hash mismatch; rebuild i2r_rlc_kcf_diagnostics: ' + name)
    hashes = record.get('source_sha256')
    if not isinstance(hashes, dict) or not hashes:
        raise RuntimeError('Native replay source hashes missing')
    for relative, expected in hashes.items():
        path = ROOT / relative
        if (pathlib.Path(relative).is_absolute() or '..' in pathlib.Path(relative).parts
                or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected):
            raise RuntimeError('Native replay source changed; rebuild i2r_rlc_kcf_diagnostics: ' + relative)
    for relative, expected in (expected_sources or {}).items():
        if (relative not in hashes or hashlib.sha256(
                (ROOT / relative).read_bytes().replace(b'\r\n', b'\n')).hexdigest() != expected):
            raise RuntimeError('Native replay does not match pinned source: ' + relative)
    runtime_dirs = record.get('runtime_dirs', [])
    if not isinstance(runtime_dirs, list) or any(not isinstance(p, str) for p in runtime_dirs):
        raise RuntimeError('Invalid native runtime directories')
    additions = [str(folder)] + [p for p in runtime_dirs if p and pathlib.Path(p).is_dir()]
    current = os.environ.get('PATH', '').split(os.pathsep)
    os.environ['PATH'] = os.pathsep.join(dict.fromkeys(additions + current))
    os.environ['PYTHONUTF8'] = '1'
    provenance = dict(source_sha256=hashes, binary_sha256=entry['sha256'],
                      backend='cmake_prebuilt', binaries=record['binaries'],
                      manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                      dependency_versions=record.get('dependency_versions', {}),
                      build_revision=record.get('build_revision'))
    return binary, provenance


def runtime_provenance():
    """返回报告用构建标识；编译器名称不等同于已通过 MSVC 验收。"""
    configured = configured_native('DetectorReplay')
    return configured[1] if configured else {'backend': 'macos_auxiliary_clang'}
