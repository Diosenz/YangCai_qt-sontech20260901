"""Prepare the existing pipeline for the 20:22 automatic-Kalman recording."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261020_204102_kcf_only_composite_teach'
text=(TEMPLATE/'analyze.py').read_text(encoding='utf-8').replace("glob('20261020_204102_195*')", "glob('20261020_202202_369*')")
start=text.index('# Only-KCF mode:');end=text.index('# Interleaved sector holdout',start)
text=text[:start]+'''# Automatic fusion is described from actual recorded gains, not assumed fixed ratios.
fusion_delta=np.linalg.norm(fq-rawq,axis=1)
pixel_error=float(np.max(abs(vis[['before_fusion_x_px','before_fusion_y_px']].to_numpy()-vis[['final_x_px','final_y_px']].to_numpy())))
weight_stats={}
for key in ['kcf_weight_x','kcf_weight_y','prior_weight_x','prior_weight_y','normalized_peak','observation_noise']:
    values=vis[key].dropna()
    weight_stats[key]=dict(n=len(values),min=float(values.min()),median=float(values.median()),max=float(values.max())) if len(values) else dict(n=0)
raw_errors=datasets['raw_KCF'].distance_mm.to_numpy();final_errors=datasets['final_vision'].distance_mm.to_numpy()
improved=final_errors<raw_errors-1e-10;worse=final_errors>raw_errors+1e-10
fusion=dict(applied_count=int(vis.fusion_applied.sum()),valid_online_count=len(vis),weights_available_count=int(vis.kcf_weight_x.notna().sum()),raw_final_pixel_max_px=pixel_error,movement_mm=g.metric(fusion_delta),geometry_rmse_change_mm=metrics['final_vision']['rmse_mm']-metrics['raw_KCF']['rmse_mm'],correlation_jump_rejected=int(vis.correlation_jump_rejected.sum()),fusion_jump_rejected=int(vis.fusion_jump_rejected.sum()),weight_stats=weight_stats,improved_count=int(improved.sum()),worsened_count=int(worse.sum()),unchanged_count=int((~improved&~worse).sum()),template_reinitialized_count=int(vis.template_reinitialized.sum()),kalman_seeded_count=int(vis.kalman_seeded.sum()))
assert fusion['applied_count']>0
pd.DataFrame(dict(frame_id=vis.frame_id,time=[stamp(t) for t in vis.camera_timestamp_ms],fusion_applied=vis.fusion_applied,kcf_weight_x=vis.kcf_weight_x,kcf_weight_y=vis.kcf_weight_y,fusion_shift_mm=fusion_delta,raw_error_mm=raw_errors,final_error_mm=final_errors,error_change_mm=final_errors-raw_errors)).to_csv(OUT/'fusion_effects.csv',index=False,encoding='utf-8-sig')
'''+text[end:]
text=text.replace("prior=pd.read_csv(tracked(OUT.parent/'20261021_163204_sim01_kcf09_composite_teach/metrics.csv'))", "prior=pd.read_csv(tracked(OUT.parent/'20261020_204102_kcf_only_composite_teach/metrics.csv'))")
(OUT/'analyze.py').write_text(text,encoding='utf-8')
for name in ['verify.py','verify_browser.cjs']:(OUT/name).write_text((TEMPLATE/name).read_text(encoding='utf-8'),encoding='utf-8')
print('Prepared automatic-Kalman analysis.')
