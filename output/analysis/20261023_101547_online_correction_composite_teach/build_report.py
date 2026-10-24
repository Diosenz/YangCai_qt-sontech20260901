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
write('index.html',page('10月23日10:15 在线纠偏｜综合示教参考四轨迹三维图','<main style="max-width:none;margin:0;padding:16px"><h2 style="margin:0">10月23日10:15 在线纠偏｜精定位与连续估计 · 四轨迹空间对照</h2><p>变位机末节随动系，单位mm。灰色/绿色均为此前综合示教参考，半径142.035340 mm；生成W点不是新实测点。蓝色TCP、橙色原始KCF保留全部有效点；紫色最终业务视觉可点击图例打开。移动后参考为事后估计，不能当独立真值。</p><p>'+controls+'</p><div id="plot" style="height:730px"></div><p class="meta">显示切换不改变误差定义：统计始终使用连续综合圆周。Z倍率不改变任何坐标或误差。可旋转、缩放、平移；<a href="report.html" target="_top">返回报告</a>。</p></main>','<script src="plotly-2.35.2.min.js"></script><script>'+viewer_js+'</script>'))
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
write('axis_fluctuations.html',page('10月23日10:15 在线纠偏｜XYZ方向误差波动','<main style="margin:0;padding:14px;max-width:none"><h2 style="margin:0">相对移动后综合示教圆的 XYZ 偏差</h2><p>测量点减去圆周最近点；无平滑、无剔除。各轴分量用于看波动，三维距离=√(ΔX²+ΔY²+ΔZ²)。图例可显示最终业务视觉；拖选放大，双击复位。</p><div id="axes"></div></main>','<script src="plotly-2.35.2.min.js"></script><script>window.axisViewerReady=Plotly.newPlot("axes",'+json.dumps(axis_traces,ensure_ascii=False)+','+json.dumps(axis_layout)+',{responsive:true,displaylogo:false,scrollZoom:true});</script>'))
# Time windows reveal startup concentration without suppressing peaks.
windows=[]
for key,d in data.items():
    relative=(d.timestamp_ms-origin)/1000
    for name,mask in [('前60秒',relative<60),('60秒以后',relative>=60)]:
        if mask.any():windows.append(dict(trajectory=labels[key],window=name,**g.metric(d.distance_mm[mask]),above_3mm=int((d.distance_mm[mask]>3).sum())))
pd.DataFrame(windows).to_csv(OUT/'time_window_metrics.csv',index=False,encoding='utf-8-sig')
# Static overview with true coordinates; XZ remains equal scale.
im=Image.new('RGB',(1700,1150),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',23);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
draw.text((45,15),'10月23日10:15 在线纠偏 在线纠偏｜精定位与连续估计｜固定综合示教参考',fill='#203345',font=font)
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
def csvtable(filename,cols,n=None):
    d=pd.read_csv(OUT/filename);return table((d.head(n) if n else d).to_dict('records'),cols)
source_table2=csvtable('point_source_metrics.csv',[('name','来源枚举'),('n','点数'),('mean_mm','平均/mm'),('rmse_mm','RMSE/mm'),('max_mm','最大/mm'),('above_3mm_count','超3mm')])
stage_table=csvtable('stage_effects.csv',[('transition','同帧阶段'),('n','点数'),('before_rmse_mm','前RMSE/mm'),('after_rmse_mm','后RMSE/mm'),('improved','偏差减少'),('worsened','偏差增加')])
phase_table=csvtable('robot_phase_timeline.csv',[('phase','阶段'),('n','点数'),('start','北京时间起'),('end','北京时间止')])
flags_table=table([dict(name=k,count=v) for k,v in st['runtime_flags'].items()],[('name','记录标志'),('count','有效在线帧数')])
transaction_table=csvtable('local_transaction_outcomes.csv',[('record_type','记录类型'),('phase','事务阶段'),('verification_kind','验证类型'),('current_pose_geometrically_verified','当前位姿几何确认'),('retained_without_geometric_verification','未通过几何仍保留'),('workpiece_pose_revision','位姿版本')],15)
local_table=table([local],[('candidate_count','候选'),('accepted_count','候选通过'),('writeback_record_count','写回记录'),('verification_record_count','通过后验记录'),('verified_success_count','新帧后验成功'),('rollback_attempted_count','通过后验回滚标志')])
comparison=[]
for directory,label in [('20261021_201400_stability10_auto_kalman_composite_teach','10-21 20:14'),('20261021_235225_seam_refinement_composite_teach','10-21 23:52'),(OUT.name,'10-23 10:15 本轮')]:
    path=OUT.parent/directory/'metrics.csv';report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for a in pd.read_csv(path).to_dict('records'):comparison.append(dict(session=label,name=labels[a['trajectory']],**a))
pd.DataFrame(comparison).to_csv(OUT/'historical_comparison.csv',index=False,encoding='utf-8-sig')
comparison_table=table(comparison,[('session','会话'),('name','轨迹'),('mean_mm','平均/mm'),('rmse_mm','RMSE/mm'),('max_mm','最大/mm'),('above_3mm_count','超3mm')])
images=pd.read_csv(OUT/'nearest_saved_peak_images.csv');image_table=table([dict(name=labels[a['trajectory']],**a) for a in images.to_dict('records')],[('name','轨迹'),('peak_time','峰值时刻'),('peak_error_mm','峰值/mm'),('image_time','最近保存图时刻'),('time_offset_ms','图减峰值/ms')])
evidence=OUT/'evidence';evidence.mkdir(exist_ok=True);gallery=[]
for stem,group in images.groupby('stem',sort=False):
    for kind in ['source','enhanced']:
        source=ROOT/'output/online_correction'/r['session']/'vision_images'/(stem+'_'+kind+'.png');shutil.copy2(source,evidence/source.name);report_hashes[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
    gallery.append('<figure><figcaption>'+html.escape(' / '.join(labels[x] for x in group.trajectory))+'</figcaption><div style="display:grid;grid-template-columns:1fr 1fr;gap:8px"><img src="evidence/'+stem+'_source.png" alt="原图"><img src="evidence/'+stem+'_enhanced.png" alt="增强图"></div></figure>')
session=ROOT/'output/online_correction'/r['session'];diag=pd.read_csv(session/'vision_diagnostics.tsv',sep='\t');vis=diag[diag.phase.eq('online_running')&diag.result_valid.eq(1)].reset_index(drop=True)
source_traces=[]
for src,color in [(1,'#009e73'),(2,'#e69f00'),(3,'#cc3344'),(4,'#8956b1')]:
    mask=vis.point_source.eq(src).to_numpy();df=data['final_vision'][mask];source_traces.append(dict(type='scattergl',mode='markers',name=st['source_labels'][str(src)],x=((df.timestamp_ms-origin)/1000).tolist(),y=df.distance_mm.tolist(),marker=dict(color=color,size=3)))
source_traces.append(dict(type='scattergl',mode='lines',name='TCP偏差',x=((data['TCP'].timestamp_ms-origin)/1000).tolist(),y=data['TCP'].distance_mm.tolist(),line=dict(color=colors['TCP'],width=1)))
write('source_diagnostics.html',page('10月23日输出来源与偏差','<main><h1>输出来源与偏差分段</h1><p>source3沿用SimulationPrediction枚举；新版图像连续模式下不单凭此名称认定由当前仿真点牵引。下图是同一时间轴，非同一焊点配对。</p><div id="sources"></div>'+source_table2+'</main>','<script src="plotly-2.35.2.min.js"></script><script>window.sourceReady=Plotly.newPlot("sources",'+json.dumps(source_traces,ensure_ascii=False)+','+json.dumps(dict(height=650,xaxis=dict(title='首条有效记录起/s'),yaxis=dict(title='三维偏差/mm'),legend=dict(orientation='h')),ensure_ascii=False)+',{responsive:true,displaylogo:false});</script>'))
segments=pd.read_csv(OUT/'point_source_segments.csv');seg=segments[segments.source.isin([3,4])].sort_values('span_ms',ascending=False).head(12)
segment_table=table(seg.to_dict('records'),[('source','来源'),('start','开始'),('end','结束'),('count','点数'),('span_ms','首末跨度/ms'),('max_mm','最大/mm')])
source_peak=data['final_vision'].loc[data['final_vision'].distance_mm.idxmax()];peak_diag=vis.loc[vis.frame_id.eq(source_peak.sample_id)].iloc[0]
peak_info={key:(None if pd.isna(peak_diag[key]) else peak_diag[key].item() if isinstance(peak_diag[key],np.generic) else peak_diag[key]) for key in ['frame_id','point_source','geometry_valid','raw_geometry_valid','guided_valid','guided_reacquisition_pending','guided_reacquired','guided_anchor_age_ms','guided_model_translation_suppressed','workpiece_pose_revision']}
write('peak_diagnostics.json',json.dumps(peak_info,ensure_ascii=False,indent=2))
body=f'''<main><span class="badge">2026-10-23 10:15:47｜schema7｜综合示教固定半径</span><h1>最新在线纠偏误差分析<br>与刚体移动后的综合原始示教轨迹对比</h1>
<p class="meta">会话 {r['session']}｜{r['summary']['start_time']} 至 {r['summary']['end_time']}（北京时间）｜末态 {r['summary']['session_state']}</p>
<div class="actions"><a href="index.html">独立三维图</a><a href="axis_fluctuations.html">XYZ偏差</a><a href="source_diagnostics.html">来源分段图</a><a href="metrics.csv">主统计</a><a href="results.json">完整数据</a></div>
<div class="cards"><div class="card"><span>TCP平均偏差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大偏差</span><b>{tcp['max_mm']:.3f} mm</b><span>{tcp['above_3mm_count']}点超3 mm</span></div><div class="card"><span>最终视觉RMSE</span><b>{final['rmse_mm']:.3f} mm</b><span>峰值{final['max_mm']:.3f} mm</span></div><div class="card"><span>新帧后验通过</span><b>{local['verified_success_count']}次</b><span>{st['retained_unverified_count']}份未几何确认仍保留</span></div></div>
<div class="lead"><b>结论：</b>TCP平均0.782 mm、RMSE 0.963 mm、最大4.034 mm，93/5791点超3 mm。原始粗KCF RMSE 1.220 mm，最终输出降至1.052 mm，但最终超3 mm点数从86增至158。预测和辅助估计来源3/4共519点（7.53%），包含最终158个超3 mm点中的128个（81.01%），尾部偏差仍主要集中在这些来源。相较10月21日23:52组，TCP RMSE由1.232降至0.963 mm，最终视觉RMSE由1.193降至1.052 mm；本轮视觉峰值反而更大。各轮条件不同，不能据此单独归因算法改进。</div>
<p class="note"><b>终态：</b>日志记录“在线纠偏在终态前被复位”，不能视为正常完整结束。在线运行与等待机器人停止时段分开，主统计不混入启动和停止等待记录。</p>
<h2>1. 数据、参考和坐标口径</h2><p>机器人原始{r['source_counts']['robot']}条、视觉诊断{r['source_counts']['diagnostics']}条。TCP保留online_running、CORFLAG=DETFLAG=S_START=1、S_HOLD=ALARM=0共{tcp['n']}点；视觉保留online_running且result_valid=1共{kcf['n']}点。保留全部有效测量/输出点及峰值，不采用上次事后剔除口径。</p>
<p>此前四组去重历史示教各25%综合，半径固定<b>142.035339635 mm</b>。本组仅用before_fusion粗KCF的三维点估计圆心和法线：初拟合后每2°取坐标中位数代表点，SVD拟合平面、最小二乘初始化圆、稳健迭代细化；再将综合原始示教圆刚体旋转和平移到该圆心/法线。不缩放，不用TCP和最终视觉优化参考，绕法线相位取最小旋转约定。全部有效点仍参与误差统计。</p><p>参考位姿由本轮粗KCF事后估计，会吸收整体位姿偏差；它不是独立真实焊缝测量。统一到变位机末节随动材料系，机器人经RBCALIB变换、逐点消除Ry(S1)Rz(S2)；视觉按同帧姿态标定投影，工具TCP系加Z=231 mm。换坐标原点不改半径和误差。当前标定复算吻合不等于历史绝对标定得到独立验证。</p>{phase_table}
<h2>2. 空间轨迹与XYZ偏差</h2><p>蓝TCP、橙融合前粗KCF、灰虚线移动前综合示教、绿移动后综合参考，紫最终业务视觉可从图例打开。支持三维、XY/XZ/YZ、Z显示倍率和两组示教独立切换。生成W点是综合圆采样，不是新实测点。倍率只改显示。</p><iframe src="index.html" title="空间轨迹" style="height:1040px"></iframe><iframe src="axis_fluctuations.html" title="XYZ偏差" style="height:990px" loading="lazy"></iframe><figure><img src="preview.png" alt="空间投影、全量误差和累计分布"></figure>
<h2>3. 示教来源及参考移动参数</h2>{source_table}{circle_table}<p>轨迹拟合圆仅作描述，所有原始点保留。变换矩阵见teach_to_current_transform.csv，原始综合圆与移动后圆分别保存供复核。</p>
<h2>4. 全量误差</h2>{metric_table}<pre>h=(p-c)·n；ρ=||(p-c)-h n||；e=√((ρ-R)²+h²)
平均=mean(e)；RMSE=√mean(e²)；XYZ偏差=测量点−连续圆最近点</pre><p>解析连续三维圆距离，按记录样本加权，不按弧长重加权。3 mm仅为描述阈值。XYZ偏差不是机器人与视觉同焊点时序配对偏差。</p>{component_table}{window_table}{event_table}
<p>TCP峰值{tcp['max_time']}；粗KCF峰值{kcf['max_time']}；最终视觉峰值{final['max_time']}。峰值事件按连续超3 mm记录归组、时间间隔超过200 ms拆分，不从峰值时间接近直接推断因果。</p>
<h2>5. 新版视觉来源、阶段与异常段</h2>{source_table2}<p>来源1几何精定位；2校正KCF估计；3沿用SimulationPrediction枚举；4原图肩部/时序辅助估计。新版图像连续链路已变化，来源3名称本身不足以证明当前仿真点拉动输出。guided_valid不等于独立原图精定位有效；raw_geometry_valid和geometry_valid也代表不同阶段。estimated共{st['estimated_count']}点，transition_limited共{st['transition_limited_count']}点。来源指标覆盖不同时间/位置，不作受控因果比较。</p>{flags_table}<p>在线位姿版本：{st['pose_revisions']}。下表是同帧阶段比较，不把fusion_applied直接解释为最终业务输出只经过卡尔曼。粗融合RMSE可能变大，几何与最终业务输出需分别检查。</p>{stage_table}
<p><a href="source_diagnostics.html">打开输出来源全量误差时间图</a>。以下为较长的来源3/4连续段；跨度不含下一帧间隔，完整表保留。</p>{segment_table}<p>最终视觉峰值同帧诊断：</p><pre>{html.escape(json.dumps(peak_info,ensure_ascii=False,indent=2))}</pre><p>峰值帧的标志只能解释当时采用何种输出，不能代替该帧图像确认真实接缝位置。</p>
<h2>6. 局部表面ICP与事务结果</h2>{local_table}<p>归档 {local['archive']}（UTC式目录时间，实际帧时间核对）。schema4局部候选{local['candidate_count']}份，其中surface_only={st['surface_only_candidate_count']}。候选通过{local['accepted_count']}、写回{local['writeback_record_count']}；新帧后验verified仅9份，另有18份retained_without_geometric_verification记录。保留表示未回滚及位姿读回保留，不能算后验几何通过，即使某些记录succeeded或verified_by_independent_observation为true，也需按phase和current_pose_geometrically_verified判断。</p><p>基础安全通过29、拒绝31；稳定保护实际评估22份、拒绝2份。表面模式与焊点模式入口不同，不能要求所有接受候选都进入同一个稳定评估，也不能用候选数代表效果。历史仅监测的评估22份，其中8份记录历史区域冲突；候选预测不等于写回后实测退化。</p>{transaction_table}<p>完整事务CSV区分writeback/verified/retained。新帧验证类型以fresh_raw_surface记录为准，不能继续称为旧版KCF焊点验收。融合前几何焊点可重放{local['raw_archive_replay_count']}帧，最大差{local['raw_archive_replay_max_mm']:.2e} mm，仅证明这些记录的投影一致。相机、标定共享，在线新帧验收不是外部真值精度认证。</p><p>另有轨迹预测ICP {arc['batch_count']}批、{arc['applied']}批应用；释放目标对同批待处理折线最大距离{arc['released_departure_max_mm']:.3f} mm。这些与修改工件位姿的局部表面ICP、TCP对综合参考偏差是不同指标。</p>
<h2>7. 峰值附近已保存图像</h2><p>保存{sampling['pairs']}对图像，覆盖{sampling['first_saved_time']}至{sampling['last_saved_time']}。最近图像非零时间差不称为峰值同帧图，不能凭邻近图断定峰值原因；无图时保留证据缺口。</p>{image_table}{''.join(gallery)}
<h2>8. 历史同口径对照和敏感性</h2>{comparison_table}<p>各轮分别由本轮粗KCF估计同一固定综合圆的当前位置，环境与算法变化不受控。与上次相比TCP平均和RMSE降低、超3 mm点减少；最终视觉最大偏差由4.767升至7.284 mm。不能用均值或局部更新次数代替峰值和尾部分析。</p><p>本轮粗KCF周向覆盖约{r['coverage_deg']:.2f}°；交错20°扇区留出RMSE {r['holdout']['pooled']['rmse_mm']:.3f} mm，仍共享传感器标定偏差。历史示教留一组半径敏感性如下，主结果半径始终固定。</p>{sens_table}
<h2>9. 验证与建议</h2><p>像素到固定站系日志复算最大差{r['verification']['final_projection_max_mm']:.2e} mm，随动坐标复算最大差{r['verification']['positioner_replay_max_mm']:.2e} mm。核对输入哈希、独立标量圆距离、逐点分量、同帧阶段、局部归档、图像来源及离线图控件。源码只辅助解释字段，缺少运行时完整版本快照；原始日志只读，生产代码未改动。</p><p>优先复查10:19:20–10:19:56附近的检测峰值、来源切换、精定位恢复与TCP峰值，结合原图判断是否错支系或辅助估计残留；其次检查18次未几何确认仍保留的位姿事务。需要受控重复运行和独立实物测量才能评价绝对精度。终态复位及停止等待行为另行核对。</p>
<div class="actions"><a href="local_transaction_outcomes.csv">全部事务</a><a href="point_source_metrics.csv">来源统计</a><a href="point_source_segments.csv">来源连续段</a><a href="historical_comparison.csv">历史对照</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">数值核验</a><a href="browser_verification.json">浏览器核验</a></div><p>复现依次运行analyze.py、build_report.py、verify_browser.cjs、verify.py。stage_analysis.py由主脚本执行。浏览器无鼠标DOM事件验证，未进行人工拖拽验收。</p><footer>沿用此前<a href="prior_notion_workflow_snapshot.json">Notion工作流保存快照</a>，本轮未刷新在线页面。2026-10-23，北京时间。</footer></main>'''
write('report.html',page('20261023 10:15 在线纠偏误差｜移动后综合原始示教',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2));print(OUT/'report.html')
