from pathlib import Path
import pandas as pd,numpy as np,json
p=Path('output/analysis/20261021_235225_seam_refinement_composite_teach');session=Path('output/online_correction/20261021_235225_324');d=pd.read_csv(session/'vision_diagnostics.tsv',sep='	');i=int(d.index[d.frame_id.eq(17927)][0]);row=d.loc[i];stamp=lambda t:pd.to_datetime(t,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%m-%d %H:%M:%S.%f')[:-3]
cols=['frame_id','camera_timestamp_ms','geometry_valid','geometry_status','geometry_ambiguous_column_ratio','point_source','degraded_frames','transition_limited','sim_x_px','sim_y_px','before_fusion_x_px','before_fusion_y_px','coarse_final_x_px','coarse_final_y_px','geometry_x_px','geometry_y_px','final_x_px','final_y_px','kcf_weight_x']
window=d[d.camera_timestamp_ms.between(row.camera_timestamp_ms-60000,row.camera_timestamp_ms+60000)].copy();window.insert(0,'time',window.camera_timestamp_ms.map(stamp));window.to_csv(p/'screenshot_drift_window.csv',index=False,encoding='utf-8-sig')
print('FRAME',json.dumps(row[cols].to_dict(),ensure_ascii=False));prev=d.loc[:i-1];last=prev[prev.geometry_valid.eq(1)].iloc[-1];print('LAST GEOM',stamp(last.camera_timestamp_ms),last[cols].to_dict());print('NEXT',d.loc[i:i+6,cols].to_string(index=False))
a=i
while a>0 and d.loc[a-1,'point_source']==3:a-=1
b=i
while b+1<len(d) and d.loc[b+1,'point_source']==3:b+=1
seg=d.loc[a:b];dif=seg[['final_x_px','final_y_px']].to_numpy()-seg[['sim_x_px','sim_y_px']].to_numpy();print('SOURCE3 SEG',a,b,len(seg),stamp(seg.camera_timestamp_ms.iloc[0]),stamp(seg.camera_timestamp_ms.iloc[-1]),'offset range',np.ptp(dif,axis=0),'sim range',np.ptp(seg[['sim_x_px','sim_y_px']],axis=0));print('window status',window.geometry_status.value_counts().to_dict())
for k in ['raw_KCF','final_vision']:
    f=pd.read_csv(p/(k+'_circle_error_components.csv'));print(k,f[f.sample_id.eq(17927)][['time','distance_mm','signed_radial_mm','signed_plane_height_mm']].to_dict('records'))
f=pd.read_csv(p/'TCP_circle_error_components.csv');ix=(f.timestamp_ms-row.camera_timestamp_ms).abs().idxmin();print('TCP nearest',f.loc[ix,['time','distance_mm']].to_dict())
l=pd.read_csv(p/'local_registration_metrics.csv');m=d.set_index('frame_id').camera_timestamp_ms;l['time']=l.frame_id.map(m).map(lambda t:stamp(t) if pd.notna(t) else 'unknown');print('LOCAL tail',l[['frame_id','time','accepted','reason']].tail(12).to_string(index=False));v=pd.read_csv(p/'local_verification_records.csv');print('VERIFIED tail',v[['time','succeeded','file']].tail(6).to_string(index=False));print('local in segment',l[l.frame_id.between(seg.frame_id.min(),seg.frame_id.max())].shape[0])
