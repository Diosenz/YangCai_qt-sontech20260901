"""Build a self-contained offline report without modifying previous analyses."""
from pathlib import Path
import hashlib, html, importlib.util, json, shutil, sys
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2];PREV=OUT.parent/'20261021_positive20_teach_synthesis'
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
spec=importlib.util.spec_from_file_location('g',ROOT/'output/analysis/20261020_evening_comparison/assets/geometry.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
library=ROOT/'output/analysis/20261020_evening_comparison/assets/plotly-2.35.2.min.js'
shutil.copy2(library,OUT/library.name)
workflow=OUT.parent/'20261021_152328_050_retest/prior_notion_workflow_snapshot.json'
shutil.copy2(workflow,OUT/workflow.name)
labels={'TCP':'TCP 实际轨迹','raw_KCF':'KCF 原始检测','final_vision':'最终业务视觉'}
colors={'TCP':'#0072b2','raw_KCF':'#d55e00','final_vision':'#8956b1'}
data={k:pd.read_csv(OUT/(k+'_circle_error_components.csv')) for k in labels}
origin=min(d.timestamp_ms.iloc[0] for d in data.values())
css='''*{box-sizing:border-box}body{margin:0;background:#f3f6fa;color:#203345;font:16px/1.75 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1320px;margin:26px auto;padding:32px 42px;background:white;border-radius:12px}h1{font-size:30px;line-height:1.4;margin:12px 0}h2{font-size:22px;border-bottom:1px solid #dce5ec;padding-bottom:8px;margin-top:35px}h3{font-size:18px}a{color:#0072b2}p{margin:12px 0}.meta,figcaption{color:#526576;font-size:14px}.badge{background:#e6f2fc;color:#00649b;border-radius:5px;padding:5px 10px;font-size:13px}.lead{background:#eaf5f0;border-left:4px solid #009e73;padding:18px 22px}.note{background:#f2f6fa;padding:14px 20px}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:22px 0}.card{border:1px solid #dce5ec;padding:16px;border-radius:8px}.card b{display:block;font-size:27px;color:#0072b2}.card span{font-size:13px;color:#526576}.actions{display:flex;flex-wrap:wrap;gap:10px}.actions a,button,select{border:1px solid #b9cbd9;border-radius:5px;background:white;padding:7px 11px;color:#17658a;font:14px system-ui;cursor:pointer}.table-wrap{overflow:auto}table{border-collapse:collapse;width:100%;font-size:14px}td,th{text-align:left;padding:9px 11px;border-bottom:1px solid #dce5ec;white-space:nowrap}th{background:#edf4fa}iframe{width:100%;border:1px solid #dce5ec;border-radius:8px}img{max-width:100%}pre{white-space:pre-wrap;background:#f4f6f8;padding:16px;font-size:14px}footer{font-size:13px;color:#526576;margin-top:30px;padding-top:16px;border-top:1px solid #dce5ec}@media(max-width:750px){main{margin:0;padding:20px 14px}.cards{grid-template-columns:repeat(2,1fr)}h1{font-size:25px}}'''
def page(title,body,scripts=''):
    return '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>'+css+'</style></head><body>'+body+scripts+'</body></html>'
def write(name,text): (OUT/name).write_text(text,encoding='utf-8')
def table(rows,cols):
    def cell(x):return html.escape(f'{x:.4f}' if isinstance(x,(float,np.floating)) else str(x))
    return '<div class="table-wrap"><table><thead><tr>'+''.join('<th>'+v+'</th>' for k,v in cols)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+cell(row.get(k,''))+'</td>' for k,v in cols)+'</tr>' for row in rows)+'</tbody></table></div>'
def circle_points(c):
    n=np.array(c['normal']);u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u);v=np.cross(n,u);a=np.linspace(0,2*np.pi,1441)
    return np.array(c['center'])+c['radius_mm']*(np.cos(a)[:,None]*u+np.sin(a)[:,None]*v)
def trace(q,label,color,dash='solid',visible=True,mode='lines'):
    return dict(type='scatter3d',mode=mode,name=label,visible=visible,x=q[:,0].tolist(),y=q[:,1].tolist(),z=q[:,2].tolist(),line=dict(color=color,width=3,dash=dash),marker=dict(color=color,size=2),hovertemplate=label+'<br>X=%{x:.5f}<br>Y=%{y:.5f}<br>Z=%{z:.5f} mm<extra></extra>')
original=pd.read_csv(OUT/'original_composite_teach.csv').to_numpy();moved=pd.read_csv(OUT/'moved_composite_teach.csv').to_numpy()
traces=[]
for key in ['TCP','raw_KCF','final_vision']:
    traces.append(trace(data[key][['x_mm','y_mm','z_mm']].to_numpy(),labels[key],colors[key],visible=True if key!='final_vision' else 'legendonly'))
traces.extend([trace(circle_points(r['original_circle']),'综合示教圆（移动前）','#777777','dash'),trace(circle_points(r['current_circle']),'综合示教圆（移动后参考）','#009e73'),trace(original,'综合示教W点（生成点）','#777777',visible=False,mode='lines+markers'),trace(moved,'移动后综合示教W点（生成点）','#009e73',visible=False,mode='lines+markers')])
span=np.ptp(np.vstack([np.c_[t['x'],t['y'],t['z']] for t in traces]),axis=0);base=span/max(span)
layout=dict(margin=dict(l=0,r=0,t=10,b=0),legend=dict(orientation='h',x=0,y=1),scene=dict(aspectmode='manual',aspectratio=dict(zip('xyz',base)),xaxis=dict(title='X / mm'),yaxis=dict(title='Y / mm'),zaxis=dict(title='Z / mm'),camera=dict(eye=dict(x=1.5,y=1.5,z=.9))),uirevision='preserve',font=dict(family='Microsoft YaHei'))
all_points=np.vstack([np.c_[t['x'],t['y'],t['z']] for t in traces]);low=all_points.min(0);high=all_points.max(0)
for i,axis in enumerate('xyz'):
    padding=(high[i]-low[i])*.04
    layout['scene'][axis+'axis'].update(range=[float(low[i]-padding),float(high[i]+padding)],autorange=False,nticks=3 if axis=='z' else 6,tickformat='.1f' if axis=='z' else '.0f')
controls='<button data-view="iso">三维</button> <button data-view="xy">XY</button> <button data-view="xz">XZ</button> <button data-view="yz">YZ</button> <label>Z显示倍率 <select id="zscale">'+''.join(f'<option value="{i}">{i}×</option>' for i in [1,5,10,20,40])+'</select></label> <span id="scale-label">当前1×，XYZ等比例</span>'
for key,label in [('original-mode','移动前综合示教'),('relocated-mode','移动后参考')]:
    controls+=' <label>'+label+' <select id="'+key+'"><option value="fit">综合圆</option><option value="actual">生成W点与连线</option><option value="points">仅生成W点</option><option value="both">W点＋圆</option></select></label>'
viewer_js='''const traces=TRACES, layout=LAYOUT;const base=BASE;window.viewerReady=Plotly.newPlot('plot',traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});
document.getElementById('zscale').addEventListener('change',e=>{let k=Number(e.target.value);Plotly.relayout('plot',{'scene.aspectratio':{x:base[0],y:base[1],z:base[2]*k}});document.getElementById('scale-label').textContent=k===1?'当前1×，XYZ等比例':'当前Z '+k+'×，仅显示放大';});
for(const [id,fit,actual] of [['original-mode',3,5],['relocated-mode',4,6]])document.getElementById(id).addEventListener('change',e=>{const k=e.target.value;Plotly.restyle('plot',{visible:[k==='fit'||k==='both',k!=='fit']},[fit,actual]);Plotly.restyle('plot',{mode:k==='points'?'markers':'lines+markers'},[actual]);});
for(const b of document.querySelectorAll('[data-view]'))b.addEventListener('click',()=>{const eyes={iso:{x:1.5,y:1.5,z:.9},xy:{x:0,y:0,z:2.2},xz:{x:0,y:-2.2,z:0},yz:{x:2.2,y:0,z:0}};Plotly.relayout('plot',{'scene.camera':{eye:eyes[b.dataset.view],up:b.dataset.view==='xy'?{x:0,y:1,z:0}:{x:0,y:0,z:1}}});});'''.replace('TRACES',json.dumps(traces,ensure_ascii=False)).replace('LAYOUT',json.dumps(layout)).replace('BASE',json.dumps(base.tolist()))
write('index.html',page('10月21日23:52 接缝精定位｜综合示教参考四轨迹三维图','<main style="max-width:none;margin:0;padding:16px"><h2 style="margin:0">10月21日23:52 接缝精定位｜精定位与连续估计 · 四轨迹空间对照</h2><p>变位机末节随动系，单位mm。灰色/绿色均为此前综合示教参考，半径142.035340 mm；生成W点不是新实测点。蓝色TCP、橙色原始KCF保留全部有效点；紫色最终业务视觉可点击图例打开。移动后参考为事后估计，不能当独立真值。</p><p>'+controls+'</p><div id="plot" style="height:730px"></div><p class="meta">显示切换不改变误差定义：统计始终使用连续综合圆周。Z倍率不改变任何坐标或误差。可旋转、缩放、平移；<a href="report.html" target="_top">返回报告</a>。</p></main>','<script src="plotly-2.35.2.min.js"></script><script>'+viewer_js+'</script>'))
write('viewer_data.json',json.dumps(dict(traces=traces,layout=layout),ensure_ascii=False))
# Directional residuals use nearest circle feet, not paired measurements.
axis_traces=[];axis_summary=[]
for i,axis in enumerate('xyz'):
    for key in labels:
        d=data[key];values=d['delta_'+axis+'_mm'];suffix='' if i==0 else str(i+1)
        axis_traces.append(dict(type='scattergl',mode='lines',name=labels[key],legendgroup=key,showlegend=i==0,visible='legendonly' if key=='final_vision' else True,x=((d.timestamp_ms-origin)/1000).tolist(),y=values.tolist(),text=d.time.tolist(),xaxis='x'+suffix,yaxis='y'+suffix,line=dict(color=colors[key],width=1.2),hovertemplate=labels[key]+'<br>%{text}<br>Δ'+axis.upper()+'=%{y:.5f} mm<extra></extra>'))
        axis_summary.append(dict(trajectory=key,axis=axis.upper(),mean_mm=float(values.mean()),rmse_mm=float(np.sqrt(np.mean(values**2))),max_absolute_mm=float(abs(values).max())))
pd.DataFrame(axis_summary).to_csv(OUT/'axis_residual_summary.csv',index=False,encoding='utf-8-sig')
axis_layout=dict(height=790,margin=dict(l=65,r=25,t=40,b=50),legend=dict(orientation='h',y=1.07),font=dict(family='Microsoft YaHei'),hovermode='x')
for i,axis in enumerate('XYZ'):
    suffix='' if i==0 else str(i+1)
    axis_layout['yaxis'+suffix]=dict(domain=[[.72,1],[.36,.64],[0,.28]][i],title='Δ'+axis+' / mm',zeroline=True)
    axis_layout['xaxis'+suffix]=dict(anchor='y'+suffix,matches='x3' if i!=2 else None,title='在线时间/s' if i==2 else '',showticklabels=i==2)
write('axis_fluctuations.html',page('10月21日23:52 接缝精定位｜XYZ方向误差波动','<main style="margin:0;padding:14px;max-width:none"><h2 style="margin:0">相对移动后综合示教圆的 XYZ 偏差</h2><p>测量点减去圆周最近点；无平滑、无剔除。各轴分量用于看波动，三维距离=√(ΔX²+ΔY²+ΔZ²)。图例可显示最终业务视觉；拖选放大，双击复位。</p><div id="axes"></div></main>','<script src="plotly-2.35.2.min.js"></script><script>window.axisViewerReady=Plotly.newPlot("axes",'+json.dumps(axis_traces,ensure_ascii=False)+','+json.dumps(axis_layout)+',{responsive:true,displaylogo:false,scrollZoom:true});</script>'))
# Time windows reveal startup concentration without suppressing peaks.
windows=[]
for key,d in data.items():
    relative=(d.timestamp_ms-origin)/1000
    for name,mask in [('前60秒',relative<60),('60秒以后',relative>=60)]:
        if mask.any():windows.append(dict(trajectory=labels[key],window=name,**g.metric(d.distance_mm[mask]),above_3mm=int((d.distance_mm[mask]>3).sum())))
pd.DataFrame(windows).to_csv(OUT/'time_window_metrics.csv',index=False,encoding='utf-8-sig')
# Static overview with true coordinates; XZ remains equal scale.
im=Image.new('RGB',(1700,1150),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',23);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
draw.text((45,15),'10月21日23:52 接缝精定位 在线纠偏｜精定位与连续估计｜固定综合示教参考',fill='#203345',font=font)
draw.text((45,55),'蓝：TCP  橙：KCF融合前  绿：移动后综合圆  灰：移动前综合圆；无测量点删减',fill='#526576',font=small)
def chart(box,series,title,equal=False):
    x0,y0,x1,y1=box;arr=np.vstack([np.c_[x,y] for x,y,c in series]);lo=arr.min(0);hi=arr.max(0);span=np.maximum(hi-lo,1e-6)*1.1;mid=(lo+hi)/2
    if equal:
        scale=min((x1-x0)/span[0],(y1-y0)/span[1]);span=np.array([(x1-x0)/scale,(y1-y0)/scale])
    lo=mid-span/2
    draw.text((x0,y0-45),title,fill='#203345',font=font)
    for j in range(6):
        x=x0+j*(x1-x0)/5;y=y1-j*(y1-y0)/5
        draw.line((x,y0,x,y1),fill='#e2e8ef');draw.line((x0,y,x1,y),fill='#e2e8ef')
        draw.text((x-18,y1+5),f'{lo[0]+span[0]*j/5:.1f}',fill='#526576',font=small)
        draw.text((x0-70,y-10),f'{lo[1]+span[1]*j/5:.1f}',fill='#526576',font=small)
    for x,y,c in series:
        xx=x0+(np.asarray(x)-lo[0])/span[0]*(x1-x0);yy=y1-(np.asarray(y)-lo[1])/span[1]*(y1-y0);draw.line(list(zip(xx,yy)),fill=c,width=2)
spatial=[(circle_points(r['original_circle']),'#777777'),(data['TCP'][['x_mm','y_mm','z_mm']].to_numpy(),colors['TCP']),(data['raw_KCF'][['x_mm','y_mm','z_mm']].to_numpy(),colors['raw_KCF']),(circle_points(r['current_circle']),'#009e73')]
chart((110,170,790,540),[(q[:,0],q[:,1],c) for q,c in spatial],'XY / mm（等比例）',True)
chart((980,170,1640,540),[(q[:,0],q[:,2],c) for q,c in spatial],'XZ / mm（等比例）',True)
chart((110,700,790,1060),[((data[k].timestamp_ms-origin)/1000,data[k].distance_mm,colors[k]) for k in ['TCP','raw_KCF']],'全部有效在线点：时间/s → 三维误差/mm')
chart((980,700,1640,1060),[(np.sort(data[k].distance_mm),np.arange(1,len(data[k])+1)/len(data[k]),colors[k]) for k in ['TCP','raw_KCF']],'距离/mm → 累计比例')
im.save(OUT/'preview.png')
main_rows=[dict(name=labels[k],**a,within2=f"{100*a['within_2mm_fraction']:.2f}%",within3=f"{100*a['within_3mm_fraction']:.2f}%") for k,a in r['metrics'].items()]
metric_table=table(main_rows,[('name','轨迹'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('p95_mm','P95/mm'),('within2','≤2 mm'),('within3','≤3 mm'),('above_3mm_count','>3 mm点数')])
comparison_table=table([dict(name=labels[a['trajectory']],**a) for a in r['comparison']],[('name','轨迹'),('previous_mean_mm','上一轮平均/mm'),('current_mean_mm','本轮平均/mm'),('previous_rmse_mm','上一轮RMSE/mm'),('current_rmse_mm','本轮RMSE/mm'),('rmse_change_mm','RMSE变化/mm'),('previous_max_mm','上一轮最大/mm'),('current_max_mm','本轮最大/mm')])
circles=[dict(name='固定综合原始示教（移动前）',**r['original_circle']),dict(name='移动后综合示教参考',**r['current_circle'])]+[dict(name=labels[k]+'拟合圆',**c) for k,c in r['fitted_circles'].items()]
circle_table=table([dict(name=c['name'],x=c['center'][0],y=c['center'][1],z=c['center'][2],radius_mm=c['radius_mm'],normal=', '.join(f'{a:.9f}' for a in c['normal'])) for c in circles],[('name','圆'),('x','圆心X/mm'),('y','圆心Y/mm'),('z','圆心Z/mm'),('radius_mm','半径/mm'),('normal','单位法线')])
ev=pd.read_csv(OUT/'over_3mm_events.csv');event_table=table([dict(name=labels[a['trajectory']],**a) for a in ev.sort_values('max_mm',ascending=False).head(15).to_dict('records')],[('name','轨迹'),('start','开始'),('end','结束'),('count','点数'),('max_mm','最大/mm'),('radial_at_peak_mm','峰值径向/mm'),('plane_at_peak_mm','峰值离面/mm')])
component_table=table([dict(name=labels[a['trajectory']],**a) for a in r['components']],[('name','轨迹'),('radial_mean_mm','径向有符号均值/mm'),('radial_rmse_mm','径向RMSE/mm'),('plane_mean_mm','离面有符号均值/mm'),('plane_rmse_mm','离面RMSE/mm'),('delta_x_rmse_mm','ΔX RMSE/mm'),('delta_y_rmse_mm','ΔY RMSE/mm'),('delta_z_rmse_mm','ΔZ RMSE/mm')])
window_table=table(windows,[('trajectory','轨迹'),('window','时段'),('n','点数'),('mean_mm','平均/mm'),('rmse_mm','RMSE/mm'),('max_mm','最大/mm'),('above_3mm','>3 mm点数')])
sens=pd.read_csv(OUT/'leave_one_teaching_out_sensitivity.csv');sens_rows=[]
for key in ['TCP','raw_KCF']:
    a=sens[sens.trajectory.eq(key)];sens_rows.append(dict(name=labels[key],mean=f'{a.mean_mm.min():.3f}–{a.mean_mm.max():.3f}',rmse=f'{a.rmse_mm.min():.3f}–{a.rmse_mm.max():.3f}',maximum=f'{a.max_mm.min():.3f}–{a.max_mm.max():.3f}'))
sens_table=table(sens_rows,[('name','轨迹'),('mean','留一组示教：平均范围/mm'),('rmse','RMSE范围/mm'),('maximum','最大范围/mm')])
source_table=table([dict(name=a['name'],n=a['W_points'],radius=a['circle']['radius_mm'],weight='25%') for a in json.loads((PREV/'results.json').read_text(encoding='utf-8'))['sources']],[('name','历史示教'),('n','W点数'),('radius','拟合半径/mm'),('weight','综合权重')])
tcp=r['metrics']['TCP'];kcf=r['metrics']['raw_KCF'];final=r['metrics']['final_vision'];f=r['fusion'];arc=r['archive'];peak=ev.sort_values('max_mm',ascending=False).iloc[0]



local=r['local_registration'];st=r['stages'];sampling=r['image_sampling'];report_hashes={}
def csvtable(filename,cols,limit=None):
    frame=pd.read_csv(OUT/filename)
    return table(frame.head(limit).to_dict('records') if limit else frame.to_dict('records'),cols)
stage_table=csvtable('stage_effects.csv',[('transition','阶段变化'),('n','同帧点数'),('before_rmse_mm','前RMSE/mm'),('after_rmse_mm','后RMSE/mm'),('movement_mean_mm','平均空间变化/mm'),('improved','偏差减小'),('worsened','偏差增大')])
source_table2=csvtable('point_source_metrics.csv',[('name','输出来源'),('n','点数'),('mean_mm','平均/mm'),('rmse_mm','RMSE/mm'),('max_mm','最大/mm'),('above_3mm_count','>3 mm点数')])
phase_table=csvtable('robot_phase_timeline.csv',[('phase','状态'),('n','记录数'),('start','起始北京时间'),('end','末条北京时间')])
segments=pd.read_csv(OUT/'point_source_segments.csv');prediction=segments[segments.source.eq(3)].sort_values('span_ms',ascending=False).head(10)
segment_table=table(prediction.to_dict('records'),[('start','预测段开始'),('end','预测段结束'),('count','点数'),('span_ms','首末跨度/ms'),('max_mm','最大/mm')])
local_table=table([local],[('candidate_count','局部候选'),('accepted_count','候选通过'),('writeback_record_count','写回记录'),('verification_record_count','后验记录'),('verified_success_count','新帧后验成功'),('rollback_attempted_count','回滚标志')])
verified_table=csvtable('local_verification_records.csv',[('time','后验时刻'),('succeeded','成功'),('independently_verified','新帧确认'),('cumulative_translation_mm','累计平移/mm'),('cumulative_rotation_deg','累计旋转/度')])
images=pd.read_csv(OUT/'nearest_saved_peak_images.csv');image_table=table([dict(name=labels[a['trajectory']],**a) for a in images.to_dict('records')],[('name','轨迹峰值'),('peak_time','峰值时间'),('peak_error_mm','峰值/mm'),('image_time','最近已保存图时间'),('time_offset_ms','图像减峰值/ms'),('frame_id','图像帧')])
evidence=OUT/'evidence';evidence.mkdir(exist_ok=True);gallery=[]
for stem,group in images.groupby('stem',sort=False):
    row=group.iloc[0]
    for kind in ['source','enhanced']:
        source=ROOT/'output/online_correction'/r['session']/'vision_images'/(stem+'_'+kind+'.png');shutil.copy2(source,evidence/source.name);report_hashes[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
    gallery.append('<figure><figcaption>'+html.escape(' / '.join(labels[k] for k in group.trajectory)+'；'+str(row.image_time))+'</figcaption><div style="display:grid;grid-template-columns:1fr 1fr;gap:8px"><img src="evidence/'+stem+'_source.png" alt="原始图"><img src="evidence/'+stem+'_enhanced.png" alt="增强图"></div></figure>')
previous=[]
for directory,label in [('20261021_185548_stability_auto_kalman_composite_teach','10-21 18:55 稳定保护'),('20261021_201400_stability10_auto_kalman_composite_teach','10-21 20:14 稳定保护1.0'),(OUT.name,'10-21 23:52 本轮精定位')]:
    path=OUT.parent/directory/'metrics.csv';report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for item in pd.read_csv(path).to_dict('records'):previous.append(dict(session=label,name=('最终业务视觉' if directory==OUT.name else '融合后视觉') if item['trajectory']=='final_vision' else labels[item['trajectory']],**item))
pd.DataFrame(previous).to_csv(OUT/'historical_comparison.csv',index=False,encoding='utf-8-sig')
comparison_table=table(previous,[('session','会话'),('name','轨迹'),('mean_mm','平均/mm'),('rmse_mm','RMSE/mm'),('max_mm','最大/mm'),('above_3mm_count','>3 mm点数')])
session=ROOT/'output/online_correction'/r['session'];diag=pd.read_csv(session/'vision_diagnostics.tsv',sep='\t');vis=diag[diag.phase.eq('online_running')&diag.result_valid.eq(1)].reset_index(drop=True)
source_labels={1:'原图精定位',2:'校正KCF估计',3:'仿真预测估计'}
stage_traces=[]
for source,color in [(1,'#009e73'),(2,'#e69f00'),(3,'#cc3344')]:
    mask=vis.point_source.eq(source).to_numpy();frame=data['final_vision'][mask]
    stage_traces.append(dict(type='scattergl',mode='markers',name=source_labels[source],x=((frame.timestamp_ms-origin)/1000).tolist(),y=frame.distance_mm.tolist(),marker=dict(color=color,size=3)))
stage_traces.append(dict(type='scattergl',mode='lines',name='TCP误差',x=((data['TCP'].timestamp_ms-origin)/1000).tolist(),y=data['TCP'].distance_mm.tolist(),line=dict(color=colors['TCP'],width=1)))
stage_layout=dict(height=650,title='输出来源与TCP偏差（同时间轴，非同焊点配对）',xaxis=dict(title='从首个有效记录起/s'),yaxis=dict(title='到移动后综合圆的距离/mm'),legend=dict(orientation='h'),font=dict(family='Microsoft YaHei'),shapes=[dict(type='line',xref='paper',x0=0,x1=1,y0=3,y1=3,line=dict(color='#777',dash='dash'))])
write('source_diagnostics.html',page('输出来源与偏差分段','<main><h1>精定位、校正KCF与仿真预测分段</h1><p>红色是仿真预测估计输出，不能视为同帧原图精定位观测。全量点保留。</p><div id="sources"></div>'+source_table2+segment_table+'</main>','<script src="plotly-2.35.2.min.js"></script><script>window.sourceReady=Plotly.newPlot("sources",'+json.dumps(stage_traces,ensure_ascii=False)+','+json.dumps(stage_layout,ensure_ascii=False)+',{responsive:true,displaylogo:false});</script>'))
body=f'''<main><span class="badge">2026-10-21 23:52 → 2026-10-22 00:02｜接缝精定位与连续估计</span>
<h1>最新在线纠偏结果分析<br>与移动后的综合原始示教轨迹对比</h1>
<p class="meta">会话：{r['session']}｜采集 {r['summary']['start_time']} 至 {r['summary']['end_time']}（北京时间）｜视觉诊断schema=4｜最终状态 reset</p>
<div class="actions"><a href="index.html">独立三维图</a><a href="axis_fluctuations.html">XYZ偏差</a><a href="source_diagnostics.html">输出来源与偏差分段</a><a href="metrics.csv">主统计CSV</a><a href="results.json">完整分析数据</a></div>
<div class="cards"><div class="card"><span>TCP平均偏差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大偏差</span><b>{tcp['max_mm']:.3f} mm</b><span>{tcp['above_3mm_count']}点超过3 mm</span></div><div class="card"><span>最终视觉RMSE</span><b>{final['rmse_mm']:.3f} mm</b><span>{final['above_3mm_count']}点超过3 mm</span></div><div class="card"><span>局部新帧后验成功</span><b>{local['verified_success_count']}次</b><span>候选通过{local['accepted_count']}次</span></div></div>
<div class="lead"><b>主要发现：</b>TCP平均0.844 mm、最大4.812 mm，304/5854点超过3 mm。最终视觉平均0.781 mm，低于粗KCF的0.826 mm，但RMSE由1.056升至1.193 mm，超3 mm点数由63增至389。仿真预测来源993点（14.07%），其中366点超3 mm，占最终输出超阈点的94.09%。应优先检查这些退化预测段；最终输出变化不能全部归因于卡尔曼。</div>
<p class="note"><b>运行状态：</b>最终记录为“在线纠偏在终态前被复位”。在线记录后进入等待机器人程序停止，末条机器人记录仍S_START=1、CORFLAG=0、DETFLAG=0；不写成正常完成，也不能仅凭日志判断复位原因或复位后的实际运动。主统计排除了启动和停止等待段。</p>
<h2>1. 固定参考、数据范围与坐标</h2>
<p>沿用四组去重历史示教等权综合的原始圆，半径固定<b>142.035339635 mm</b>。只用融合前粗KCF（before_fusion）估计圆心和法线，对综合示教作刚体移动；不缩放、不用TCP或最终视觉优化参考。移动后参考为事后估计，<b>不是独立测量的真实焊缝</b>。新几何精定位数据不改变主参考口径。</p>
<p>机器人记录{r['source_counts']['robot']}条，视觉诊断{r['source_counts']['diagnostics']}条。TCP筛选online_running、CORFLAG=DETFLAG=S_START=1、S_HOLD=ALARM=0，保留{tcp['n']}点；视觉筛选online_running且result_valid=1，保留{kcf['n']}点。估计来源有效输出全部保留并标识，不平滑、不删峰值。</p>
<p>统一到变位机末节随动/材料坐标系：TCP经RBCALIB变换并逐点消除Ry(S1)Rz(S2)，像素用同帧姿态和相机标定投影；工具TCP系转换回末节系加Z=231 mm。改变坐标原点不改变误差。当前标定复算吻合不等于历史标定独立验证。</p>{phase_table}
<h2>2. 三维轨迹、投影与XYZ偏差</h2><p>蓝TCP、橙融合前粗KCF、灰虚线综合原始示教、绿移动后参考。紫色最终业务视觉可从图例开启；它包含精定位、连续性处理及估计来源，不能标成单纯卡尔曼融合后KCF。W点是历史综合圆生成点。</p>
<iframe src="index.html" title="四轨迹三维图" style="height:1040px"></iframe><iframe src="axis_fluctuations.html" title="XYZ偏差" style="height:990px" loading="lazy"></iframe><figure><img src="preview.png" alt="空间投影、偏差与累计分布"></figure>
<h2>3. 综合原始示教来源与移动参数</h2>{source_table}{circle_table}<p>四组各25%；半径固定。圆心和法线对齐采用最小旋转，绕法线相位不由圆几何确定。各轨迹拟合圆只描述形状，不替换测量点。刚体变换保存在teach_to_current_transform.csv。</p>
<h2>4. 主误差统计</h2>{metric_table}<pre>h=(p-c)·n；ρ=||(p-c)-h·n||；e=√((ρ-R)²+h²)
平均=mean(e)；RMSE=√mean(e²)；XYZ偏差=测量点−参考圆最近点</pre><p>使用连续解析三维圆最近距离；按记录样本加权，不按弧长/时间重加权。3 mm为描述阈值，不是自动认定的工艺验收线。XYZ偏差不是TCP与视觉同焊点配对误差。</p>
<h2>5. 新版视觉链路：逐阶段与来源分组</h2><p>schema4区分before_fusion（粗KCF）、coarse_final（粗融合）、geometry（原图精定位）、final（最终业务输出）。粗跟踪fusion_applied={f['applied_count']}/{kcf['n']}，不能证明所有最终输出只经历卡尔曼。下表使用同一参考和同帧子集。</p>{stage_table}
<p>raw_to_coarse：粗KCF→粗融合；raw_to_geometry：仅精定位有效的4159帧；geometry_to_final：同4159帧的精定位→最终输出；raw_to_final：全部有效帧。精定位有效子集的RMSE由粗KCF 0.889→精定位0.587→最终0.600 mm，但不能外推到退化帧。最终全量RMSE较粗KCF增加约12.94%。</p>{source_table2}
<p>来源1=原图精定位；2=校正KCF估计；3=仿真预测估计。estimated共{st['estimated_count']}点、transition_limited共{st['transition_limited_count']}点。不同来源覆盖不同时间位置，来源分组不是受控因果实验。</p><p><a href="source_diagnostics.html">打开按来源着色的全量误差时间图</a>。下表为持续较长的仿真预测段，跨度不包含下一帧间隔。</p>{segment_table}
<h2>6. 分量、峰值与历史同口径对照</h2>{component_table}{window_table}{event_table}<p>TCP峰值：{tcp['max_time']}；粗KCF：{kcf['max_time']}；最终视觉：{final['max_time']}。相邻超3 mm记录按事件归段，超过200 ms间隔拆分。时间相近不能单独证明视觉输出导致随后TCP偏差。</p>{comparison_table}<p>比20:14组，TCP平均0.829→0.844 mm、RMSE 0.954→1.232 mm、最大2.784→4.812 mm。各组分别用本组粗KCF定位同一综合参考；工件、算法和运行状态未受控，不作单因素归因。</p>
<h2>7. 在线局部表面ICP证据</h2>{local_table}<p>局部归档：{local['archive']}（UTC式目录时间，帧时间匹配）。52候选中基础安全25通过、27拒绝；25次稳定评估中24通过、1拒绝。25次history_enforced=false，其中7次记录历史区域冲突，属于候选预测而非写回后实测退化。</p><p><b>24次候选通过，仅19份写回及19份独立新帧后验成功记录。</b>差额5次不能补算成功，原因需结合事件进一步核对。回滚标志0份仅描述本目录记录。</p>{verified_table}<p>新版局部独立观测来自原图geometry，不是粗KCF。{local['raw_archive_replay_count']}个不同帧重投影与归档坐标最大差{local['raw_archive_replay_max_mm']:.2e} mm。“独立新帧”仍共享相机和标定，不是外部真值认证。</p><p>另有轨迹预测ICP {arc['batch_count']}批、{arc['applied']}批应用，与局部表面ICP不同；释放目标对同批待处理折线最大偏离{arc['released_departure_max_mm']:.3f} mm，也不是TCP到参考圆误差。</p>
<h2>8. 已保存图像</h2><p>保存{sampling['pairs']}对原始/增强图像。选择各峰值最近已保存图，非零时间差不称峰值同帧图，不能据邻近图确定峰值原因。</p>{image_table}{''.join(gallery)}
<h2>9. 敏感性、验证与建议</h2><p>粗KCF周向覆盖约{r['coverage_deg']:.2f}°；交错20°扇区留出RMSE {r['holdout']['pooled']['rmse_mm']:.3f} mm。共享标定偏差仍存在，参考姿态可能吸收整体误差，不承诺绝对精度。</p>{sens_table}<p>最终像素投影与固定站系日志最大差{r['verification']['final_projection_max_mm']:.2e} mm，末节随动系复算最大差{r['verification']['positioner_replay_max_mm']:.2e} mm。输入哈希、独立标量距离、同帧比较、局部geometry及无鼠标浏览器控件分别核验。源码仅解释字段语义，缺乏当时可执行版本完整快照。</p><p>优先回查仿真预测持续退化段、原图精定位失效原因、仿真先验偏差及连续性限制；将精定位成功帧收益与退化段尾部问题分开评价。其次检查5次候选通过但缺少写回/后验的事务，以及停止等待后的复位流程。需要完整图像回放和事件日志才能进一步判断原因。</p>
<div class="actions"><a href="point_source_metrics.csv">来源统计</a><a href="point_source_segments.csv">来源连续段</a><a href="stage_effects.csv">阶段比较</a><a href="local_registration_metrics.csv">局部候选</a><a href="local_verification_records.csv">后验记录</a><a href="historical_comparison.csv">历史对照</a><a href="over_3mm_events.csv">偏差事件</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">数值复核</a><a href="browser_verification.json">浏览器检查</a></div><p>复现依次运行analyze.py、build_report.py、verify_browser.cjs、verify.py；stage_analysis.py由分析脚本调用。原始日志只读，新增文件仅在本目录。浏览器通过无鼠标DOM事件验证，未进行人工拖拽验收。</p><footer>沿用<a href="prior_notion_workflow_snapshot.json">Notion保存快照</a>，本轮未刷新在线页面。报告日期2026-10-22（北京时间）。</footer></main>'''
write('report.html',page('20261021 23:52 接缝精定位与连续估计｜移动后综合示教对比',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2))
print(str(OUT/'report.html'))
