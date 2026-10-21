"""Check the patch preserves existing work and the external scene file."""
from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent; ROOT=OUT.parents[2]
backup=ROOT/'.codex_backups/robotsim_tcp_alignment_20261021'
before=(backup/'tests/CMakeLists.txt').read_text(encoding='utf-8-sig')
after=(ROOT/'tests/CMakeLists.txt').read_text(encoding='utf-8-sig')
start=after.index('i2r_rlc_add_test(i2r_rlc_mh12_inverse_kinematics_tests')
end=after.index('i2r_rlc_add_test(i2r_rlc_digital_twin_tests',start)
assert after[:start]+after[end:]==before
old=json.loads((ROOT/'output/analysis/20261021_full_chain_audit/input_hashes.json').read_text(encoding='utf-8'))
protected=['UI/src/RuntimeWorkerAdapters.cpp','UI/src/RuntimeLocalRegistration.cpp','config/calibration/SENSORPARA.txt','config/calibration/RBCALIB.DAT']
checked=[]
for relative in protected:
    path=ROOT/relative
    h=old.get(str(path))
    if h is not None:
        assert hashlib.sha256(path.read_bytes()).hexdigest()==h,relative
        checked.append(relative)
metadata=json.loads((OUT/'replay_metadata.json').read_text(encoding='utf-8'))
scene=Path(metadata['scene_file'])
assert hashlib.sha256(scene.read_bytes()).hexdigest()==metadata['scene_sha256']
for relative in ['DigitalTwin/src/MotomanMh12InverseKinematics.cpp','DigitalTwin/include/MotomanMh12InverseKinematics.h','tests/test_mh12_inverse_kinematics.cpp']:
    data=(ROOT/relative).read_bytes()
    assert data.startswith(b'\xef\xbb\xbf'),relative
    data.decode('utf-8-sig')
result=dict(cmake_existing_changes_preserved=True,protected_files_checked=checked,robotsim_scene_unchanged=True,cpp_utf8_bom=True,backups=str(backup))
(OUT/'scope_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
