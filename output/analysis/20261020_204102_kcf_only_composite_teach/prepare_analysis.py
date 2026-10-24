"""Reuse the checked analysis pipeline with this run's actual operating evidence."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261021_163204_sim01_kcf09_composite_teach'
text=(TEMPLATE/'analyze.py').read_text(encoding='utf-8')
text=text.replace("glob('20261021_163204_243*')", "glob('20261020_204102_195*')")
start=text.index('# Verify the recorded manual');end=text.index('# Interleaved sector holdout',start)
text=text[:start]+'''# Only-KCF mode: prove both the recorded flag and pixel/spatial identity.
fusion_delta=np.linalg.norm(fq-rawq,axis=1)
pixel_error=float(np.max(abs(vis[['before_fusion_x_px','before_fusion_y_px']].to_numpy()-vis[['final_x_px','final_y_px']].to_numpy())))
fusion=dict(applied_count=int(vis.fusion_applied.sum()),valid_online_count=len(vis),weights_available_count=int(vis.kcf_weight_x.notna().sum()),raw_final_pixel_max_px=pixel_error,movement_mm=g.metric(fusion_delta),geometry_rmse_change_mm=metrics['final_vision']['rmse_mm']-metrics['raw_KCF']['rmse_mm'],correlation_jump_rejected=int(vis.correlation_jump_rejected.sum()),fusion_jump_rejected=int(vis.fusion_jump_rejected.sum()))
assert fusion['applied_count']==0 and pixel_error==0 and fusion_delta.max()==0
pd.DataFrame(dict(frame_id=vis.frame_id,time=[stamp(t) for t in vis.camera_timestamp_ms],fusion_shift_mm=fusion_delta,raw_error_mm=datasets['raw_KCF'].distance_mm,final_error_mm=datasets['final_vision'].distance_mm)).to_csv(OUT/'fusion_effects.csv',index=False,encoding='utf-8-sig')
'''+text[end:]
text=text.replace("prior=pd.read_csv(tracked(PREV/'metrics.csv'))", "prior=pd.read_csv(tracked(OUT.parent/'20261021_163204_sim01_kcf09_composite_teach/metrics.csv'))")
start=text.index("archive=ROOT/'output/online_local_registration/");end=text.index("pd.DataFrame([dict(job_line=",start)
text=text[:start]+'''# These are trajectory prediction batches, not local surface registration/writeback.
batches=read(SESSION/'online_icp_batches.txt');points=read(SESSION/'online_icp_points.txt');effects=[]
for bi,grp in points.groupby('batch_index',sort=False):
    released=grp[grp.kind.eq('released_target')][['x_mm','y_mm','z_mm']].to_numpy()
    pending=grp[grp.kind.eq('raw_pending')][['x_mm','y_mm','z_mm']].to_numpy()
    if len(released) and len(pending)>1:
        effects.append(dict(batch_index=int(bi),released_count=len(released),**g.metric(g.closest_polyline(released,pending)[0])))
effects=pd.DataFrame(effects).merge(batches,on='batch_index')
effects.to_csv(OUT/'trajectory_icp_effects.csv',index=False,encoding='utf-8-sig')
applied=effects[effects.applied.eq(1)]
archive=dict(batch_count=len(batches),attempted=int(batches.attempted.sum()),applied=int(batches.applied.sum()),not_applied=int(batches.applied.eq(0).sum()),enabled=int(batches.enabled.sum()),released_departure_max_mm=float(applied.max_mm.max()),batch_mean_departure_median_mm=float(applied.mean_mm.median()),semantic='online trajectory prediction ICP, not surface-localization SET_POSE evidence')
assert len(batches)==int(s['icp_batch_count'])
'''+text[end:]
text=text.replace("archive=dict(candidates=len(archive_rows),accepted=sum(bool(a['accepted']) for a in archive_rows),writebacks=len(writebacks),verified=sum(bool(a['succeeded']) and bool(a['independently_verified']) for a in verified))",'archive=archive')
text=text.replace("raw_archive_replay_count=len(replay),raw_archive_replay_max_mm=float(max(replay)),fusion_formula_max_px=fusion['formula_max_abs_px']", "raw_final_pixel_max_px=pixel_error,raw_final_spatial_max_mm=float(fusion_delta.max())")
text=text.replace("'Vision/src/track/KcfTracker.cpp'", "'Vision/src/track/KcfTracker.cpp','Control/src/OnlineCorrectionExceptionProcessor.cpp','docs/online_local_registration.md'")
(OUT/'analyze.py').write_text(text,encoding='utf-8')
verify=(TEMPLATE/'verify.py').read_text(encoding='utf-8').replace('[5855,7597,7597,1441,1441,1368,1368]', "[r['metrics']['TCP']['n'],r['metrics']['raw_KCF']['n'],r['metrics']['final_vision']['n'],1441,1441,len(a),len(b)]")
(OUT/'verify.py').write_text(verify,encoding='utf-8')
(OUT/'verify_browser.cjs').write_text((TEMPLATE/'verify_browser.cjs').read_text(encoding='utf-8'),encoding='utf-8')
print('Prepared new run pipeline; existing outputs unchanged.')
