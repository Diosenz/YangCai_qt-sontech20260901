from pathlib import Path
import hashlib, json

p = Path('docs/online_local_registration.md')
s = p.read_text(encoding='utf-8-sig')
s = s.replace('恢复成功后仍暂停本会话自动更新，避免重复错误匹配；恢复结果未知禁止重复恢复。',
'''2026-10-22 起，恢复指令、回读和事务归档均成功后，保留上一已验证位姿、清空旧窗口/历史并升代；经过冻结的 minimum_match_interval_ms 冷却后，仅恢复完成之后的新曝光及递增帧号可重新建立完整窗口。恢复结果未知、仍有 pending 或归档失败继续暂停，禁止重复恢复。会话累计预算及初始基线不重置；恢复到尚未通过新帧几何验证的初始位姿时，仍需独立焊点重新建立首次可信基准。''', 1)
old = '表面支路写回前冻结候选留出基线与验收配置，且须确认该基线满足绝对残差及双向覆盖门槛。`COnlineLocalSurfaceVerification` 每方向分别比较完整最近邻均值，不允许一向改善掩盖另一向恶化，也不允许用稳健双向残差基线与完整均值新帧混用。'
new = '''表面支路写回前冻结候选留出基线与验收配置，且须确认该基线满足绝对残差及双向覆盖门槛。2026-10-22 修正后，留出基线仅作写前准入；新帧真实表面与重新求交的当前仿真切片仍须分别满足两向完整最近邻均值的绝对上限及覆盖率，但不再将这些移动切片的均值直接与旧留出均值作0.1 mm恶化比较。

相对恶化改由 `compareFreshSurface` 使用同一新帧原图实测点、冻结训练虚拟云和本次最终修正共同计算。只从写前训练虚拟云的12点近邻估计局部面；留出、新实测及估计焊点不拟合面。邻域必须有二维展开且满足既有平面性/法向支持要求；同一个局部面在原位和应用候选后均须处于既有 coverage_distance_mm 有界邻域内。相同新实测点与相同局部面身份分别计算修正前/后的平均绝对点到面距离，至少24点及既有覆盖比例通过，再检查当前误差不超过绝对残差上限、也不比同帧原位误差恶化超过 maximum_independent_worsening_mm。不会以已接受的新帧抬升门限或重新拟合候选。

这是有界局部平面近似，不是 CAD 反事实重新求交，也不证明全工件精度；新切片离开冻结几何支持范围、法向退化或原图提取失败时仍不予晋升。真实现场效果需要新一轮在线复测；旧归档没有后验真实点云，不能为旧三帧补造配对指标。'''
assert old in s
s = s.replace(old, new, 1)
s = s.replace('连续三份无效/超限表面或五秒未确认继续沿用原受保护恢复与暂停机制，不改变未知 SET_POSE 结果禁止重试等事务边界。',
'''连续三份无效/超限表面或五秒未确认仍触发受保护恢复；恢复确认后按前述冷却/新曝光边界重建窗口，未知 SET_POSE 结果禁止重试等事务边界不变。后验归档追加 comparison_kind、配对前后误差、支持点数/比例及新帧真实/当前虚拟点云；冻结参考对应同一尝试归档中的 solve_virtual_parent_mm。''', 1)
p.write_text(s, encoding='utf-8-sig', newline='\r\n')

backup = Path('output/backups/local_surface_recovery_20261022')
files = [str(f.relative_to(backup)).replace('\\', '/') for f in backup.rglob('*') if f.is_file()]
files += ['UI/include/OnlineLocalRegistrationRecovery.h']
for name in files:
    if name.endswith(('.h', '.cpp')):
        path = Path(name)
        content = path.read_text(encoding='utf-8-sig')
        path.write_text(content, encoding='utf-8-sig', newline='\r\n')
report = {'files': files,
    'before_sha256': {name: hashlib.sha256((backup/name).read_bytes()).hexdigest() for name in files if (backup/name).exists()},
    'after_sha256': {name: hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in files},
    'notion_current_access': 'public root and two workflow subpages unavailable via web tool; used saved workflow snapshot',
    'notion_snapshot': 'output/analysis/20261021_kcf_right_bias_investigation/notion_workflow_snapshot.json',
    'baseline_tests': {'localization_surface_validation': 'passed', 'online_local_surface_verification': 'passed', 'local_registration_writeback_safety': 'passed'},
    'live_devices_started': False,
    'computer_use_before_user_prohibition': 'initialized skill and enumerated windows only; no input actions',
    'computer_use_after_user_prohibition': False}
Path('output/analysis/20261022_local_surface_recovery/file_integrity.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print('Backed up and updated files:', len(files))
