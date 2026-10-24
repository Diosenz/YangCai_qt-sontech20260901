from pathlib import Path
import pandas as pd,numpy as np,json,hashlib,html
p=Path('output/analysis/20261021_235225_seam_refinement_composite_teach');out=p/'exclusion_sensitivity';out.mkdir(exist_ok=True)
s=Path('output/online_correction/20261021_235225_324/vision_diagnostics.tsv');d=pd.read_csv(s,sep='\t');v=d[d.phase.eq('online_running')&d.result_valid.eq(1)].reset_index(drop=True);i=int(v.index[v.frame_id.eq(17927)][0])
a=b=i
while a>0 and v.loc[a-1,'point_source']==3:a-=1
while b+1<len(v) and v.loc[b+1,'point_source']==3:b+=1
c=e=i
while c>0 and v.loc[c-1,'geometry_valid']==0:c-=1
while e+1<len(v) and v.loc[e+1,'geometry_valid']==0:e+=1
stamp=lambda t:pd.to_datetime(t,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%m-%d %H:%M:%S.%f')[:-3]
scenarios=[dict(key='all',label='全量有效记录',mask=np.zeros(len(v),bool)),dict(key='screenshot_prediction_segment',label='剔除截图所在连续仿真预测段',mask=v.index.to_series().between(a,b).to_numpy()),dict(key='screenshot_geometry_outage',label='剔除截图所在整段精定位失效',mask=v.index.to_series().between(c,e).to_numpy()),dict(key='all_prediction_sources',label='剔除全程所有仿真预测来源帧',mask=v.point_source.eq(3).to_numpy())]
rows=[];definitions=[];hashes={str(s.resolve()):hashlib.sha256(s.read_bytes()).hexdigest()}
for scenario in scenarios:
    mask=scenario['mask'];selected=v[mask];definitions.append(dict(key=scenario['key'],label=scenario['label'],removed_visual_count=int(mask.sum()),start=stamp(selected.camera_timestamp_ms.iloc[0]) if len(selected) else None,end=stamp(selected.camera_timestamp_ms.iloc[-1]) if len(selected) else None,start_frame=int(selected.frame_id.iloc[0]) if len(selected) else None,end_frame=int(selected.frame_id.iloc[-1]) if len(selected) else None,contiguous=scenario['key']!='all_prediction_sources'))
    for key in ['raw_KCF','final_vision','TCP']:
        source=p/(key+'_circle_error_components.csv');f=pd.read_csv(source);hashes[str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
        if key=='TCP':
            # Diagnostic matching only for the two explicitly contiguous clock windows.
            if scenario['key']=='all_prediction_sources':continue
            removed=f.timestamp_ms.between(selected.camera_timestamp_ms.iloc[0],selected.camera_timestamp_ms.iloc[-1]).to_numpy() if len(selected) else np.zeros(len(f),bool)
        else:
            assert np.array_equal(f.sample_id.to_numpy(),v.frame_id.to_numpy());removed=mask
        kept=f[~removed];err=kept.distance_mm.to_numpy();peak=kept.loc[kept.distance_mm.idxmax()]
        rows.append(dict(scenario=scenario['key'],label=scenario['label'],trajectory=key,removed=int(removed.sum()),n=len(kept),mean_mm=float(err.mean()),rmse_mm=float(np.sqrt(np.mean(err**2))),max_mm=float(err.max()),p95_mm=float(np.percentile(err,95)),above_3mm_count=int((err>3).sum()),max_time=str(peak.time)))
        if key!='TCP':pd.DataFrame(dict(frame_id=v.frame_id,camera_timestamp_ms=v.camera_timestamp_ms,excluded=mask)).to_csv(out/(scenario['key']+'_frame_mask.csv'),index=False,encoding='utf-8-sig')
pd.DataFrame(rows).to_csv(out/'metrics.csv',index=False,encoding='utf-8-sig');(out/'window_definitions.json').write_text(json.dumps(definitions,ensure_ascii=False,indent=2),encoding='utf-8')
ref=p/'moved_synthesized_circle.json';hashes[str(ref.resolve())]=hashlib.sha256(ref.read_bytes()).hexdigest();hashes[str(Path(__file__).resolve())]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();(out/'input_hashes.json').write_text(json.dumps(hashes,ensure_ascii=False,indent=2),encoding='utf-8')
result=dict(reference=json.loads(ref.read_text(encoding='utf-8')),reference_refitted=False,definitions=definitions,metrics=rows,note='事后剔除敏感性分析，不替代全量性能。仅剔除KCF点时TCP和最终输出的原统计保持不变；同窗剔除三轨迹另列。')
(out/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
cols=['label','trajectory','removed','n','mean_mm','rmse_mm','max_mm','above_3mm_count','max_time'];heads=['筛选口径','轨迹','剔除点数','保留点数','平均/mm','RMSE/mm','最大/mm','超3mm点数','峰值时间']
table='<table><tr>'+''.join('<th>'+x+'</th>' for x in heads)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(f'{r[k]:.4f}' if isinstance(r[k],float) else str(r[k]))+'</td>' for k in cols)+'</tr>' for r in rows)+'</table>'
body='<h1>截图帧17927异常段剔除敏感性分析</h1><p>固定此前移动后综合示教圆，半径142.035339635 mm，不重新拟合参考、不改原始日志。保留点仍采用连续圆解析距离。</p><p>只删除粗KCF显示/统计点不会改变已执行TCP，也不会重算已经产生的最终输出。下表对最终视觉及TCP的过滤是同时间窗口的独立描述，不是反事实重放。</p><pre>'+html.escape(json.dumps(definitions,ensure_ascii=False,indent=2))+'</pre>'+table+'<p>剔除全程仿真预测来源是额外敏感性口径，并不代表仅截图这一段。预测来源帧的before_fusion仍是粗KCF数据，final则是预测输出。</p><p>该分析由已观察异常选择排除范围，只用于定位影响，不替代全量指标或实际运行质量。时间为北京时间，10-22指2026-10-22。</p><p><a href="../report.html">全量主报告</a> | <a href="metrics.csv">统计CSV</a> | <a href="window_definitions.json">剔除范围</a></p>'
(out/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>帧17927异常段剔除分析</title><style>body{font:16px/1.7 system-ui,Microsoft YaHei;margin:30px;color:#203345}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}th{background:#eef4fa}pre{white-space:pre-wrap}</style>'+body,encoding='utf-8')
# Independent scalar verification from coordinates for every retained measurement.
circle=result['reference'];center=np.array(circle['center']);normal=np.array(circle['normal']);radius=circle['radius_mm']
for row in rows:
    f=pd.read_csv(p/(row['trajectory']+'_circle_error_components.csv'));sc=next(x for x in scenarios if x['key']==row['scenario']);sel=v[sc['mask']]
    removed=sc['mask'] if row['trajectory']!='TCP' else (f.timestamp_ms.between(sel.camera_timestamp_ms.iloc[0],sel.camera_timestamp_ms.iloc[-1]).to_numpy() if len(sel) else np.zeros(len(f),bool))
    q=f.loc[~removed,['x_mm','y_mm','z_mm']].to_numpy()-center;h=q@normal;rho=np.sqrt(np.maximum(0,(q*q).sum(1)-h*h));err=np.sqrt((rho-radius)**2+h*h)
    assert abs(err.mean()-row['mean_mm'])<1e-10 and abs(np.sqrt(np.mean(err**2))-row['rmse_mm'])<1e-10 and abs(err.max()-row['max_mm'])<1e-10
for name,digest in hashes.items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest
(out/'verification.json').write_text(json.dumps(dict(status='passed',independent_coordinate_checks=len(rows),input_hashes_verified=len(hashes),fixed_reference=True,original_files_modified=False),indent=2),encoding='utf-8')
print(json.dumps(dict(definitions=definitions,metrics=rows),ensure_ascii=False,indent=2))
