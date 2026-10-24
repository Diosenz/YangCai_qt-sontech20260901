from pathlib import Path
import difflib, hashlib, json
import xml.etree.ElementTree as ET

root = Path('output/analysis/20261022_local_surface_recovery')
backup = Path('output/backups/local_surface_recovery_20261022')
integrity = json.loads((root/'file_integrity.json').read_text(encoding='utf-8'))
patches = []
encoding_errors = []
for name in integrity['files']:
    path = Path(name)
    data = path.read_bytes()
    data.decode('utf-8-sig')
    if path.suffix in {'.h', '.cpp'} and not data.startswith(b'\xef\xbb\xbf'):
        # 新文件归一化为项目要求的 BOM/CRLF；不改变已编译的源内容。
        path.write_text(data.decode('utf-8-sig'), encoding='utf-8-sig', newline='\r\n')
        data = path.read_bytes()
    text = data.decode('utf-8-sig')
    if '\ufffd' in text:
        encoding_errors.append(name)
    previous = (backup/name).read_text(encoding='utf-8-sig') if (backup/name).exists() else ''
    patches.extend(difflib.unified_diff(previous.splitlines(True), text.replace('\r\n', '\n').splitlines(True),
        fromfile='before/'+name, tofile='after/'+name))
    integrity['after_sha256'][name] = hashlib.sha256(data).hexdigest()
integrity['notion_current_access'] = 'read current public root, collaboration workflow, bug-fix workflow and general preferences via public loadPageChunk endpoint; no Computer Use'
integrity['notion_current_response_files'] = [str(p) for p in root.glob('notion_*.json')]
integrity['encoding_errors'] = encoding_errors
integrity['missing_source_files'] = [name for name in integrity['files'] if not Path(name).exists()]
(root/'task.patch').write_text(''.join(patches), encoding='utf-8')
(root/'file_integrity.json').write_text(json.dumps(integrity, ensure_ascii=False, indent=2), encoding='utf-8')
report = {'build': {'result': 'passed', 'compiler': 'VS2022 MSVC x64 Developer environment',
    'existing_build_tree': 'out/build/qt-debug', 'log': 'build.log',
    'main_binary': 'out/build/qt-debug/UI/i2r_rlc.exe'},
    'review': {'baseline': str(backup), 'task_diff': 'task.patch', 'scope': integrity['files'],
        'no_oldproject_RobotSim_Vision_Control_changes_by_this_task': True,
        'source_files_removed': [], 'encoding_errors': encoding_errors},
    'validation_limits': [
        'No physical camera, robot or live RobotSim welding loop was started.',
        'Paired posterior metric is a bounded frozen local-plane approximation, not CAD counterfactual reintersection.',
        'The old run lacks fresh posterior point clouds; synthetic paired evidence is not presented as the old run physical truth.',
        'Unknown writes, readback/archive failures, insufficient local support and absolute limits continue to reject verification.'],
    'computer_use_after_prohibition': False}
if (root/'tests.xml').exists():
    tests = ET.parse(root/'tests.xml').getroot().findall('.//testcase')
    report['tests'] = [{'name': t.attrib['name'], 'time_seconds': t.attrib.get('time'),
        'passed': t.find('failure') is None and t.find('error') is None} for t in tests]
    report['selected'] = len(tests)
    report['passed'] = sum(t['passed'] for t in report['tests'])
(root/'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({'files': len(integrity['files']), 'encoding_errors': encoding_errors,
    'selected': report.get('selected'), 'passed': report.get('passed')}, ensure_ascii=False))
