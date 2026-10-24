from pathlib import Path
import json
root=Path.cwd();out=root/'output/analysis/20261021_235225_seam_refinement_composite_teach';old=out.parent/'20261021_201400_stability10_auto_kalman_composite_teach'
cfg=dict(prefix='20261021_235225_324',directory=out.name,label='23:52 接缝精定位与连续估计',archive='20261021_155225_278_session_1')
(out/'run_config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
s=(old/'analyze.py').read_text(encoding='utf-8').replace('20261021_201400_950*','20261021_235225_324*')
s=s.replace("strftime('%H:%M:%S.%f')","strftime('%m-%d %H:%M:%S.%f')")
s=s.replace('raw=g.external_rotation', 'geometry=g.external_rotation(project(d.geometry_x_px,d.geometry_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)\ncoarse=g.external_rotation(project(d.coarse_final_x_px,d.coarse_final_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)\nraw=g.external_rotation',1)
s=s.replace('raw[lookup[fid]]-[0,0,231]', 'geometry[lookup[fid]]-[0,0,231]')
s=s.replace('20261020_204102_kcf_only_composite_teach/metrics.csv', '20261021_201400_stability10_auto_kalman_composite_teach/metrics.csv')
s=s.replace("for code in ['Vision/src/geometry", "for code in ['Vision/src/track/OnlineSeamContinuity.cpp','Vision/src/track/LegacyOnlineKcfDetector.cpp','UI/src/RuntimeWorkerAdapters.cpp','Vision/src/geometry")
s=s.replace('public URL supplied this turn could not be read','saved workflow reused; live Notion not refreshed this run')
(out/'analyze.py').write_text(s,encoding='utf-8')
for name in ['verify.py','verify_browser.cjs']:(out/name).write_text((old/name).read_text(encoding='utf-8'),encoding='utf-8')
print(out)
