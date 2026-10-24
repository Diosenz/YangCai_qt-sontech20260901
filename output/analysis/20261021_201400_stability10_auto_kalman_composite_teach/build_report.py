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
labels={'TCP':'TCP 实际轨迹','raw_KCF':'KCF 原始检测','final_vision':'卡尔曼融合后视觉'}
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
write('index.html',page('20:14 稳定保护1.0｜综合示教参考四轨迹三维图','<main style="max-width:none;margin:0;padding:16px"><h2 style="margin:0">20:14 稳定保护1.0｜自动卡尔曼 · 四轨迹空间对照</h2><p>变位机末节随动系，单位mm。灰色/绿色均为此前综合示教参考，半径142.035340 mm；生成W点不是新实测点。蓝色TCP、橙色原始KCF保留全部有效点；紫色卡尔曼融合后视觉可点击图例打开。移动后参考为事后估计，不能当独立真值。</p><p>'+controls+'</p><div id="plot" style="height:730px"></div><p class="meta">显示切换不改变误差定义：统计始终使用连续综合圆周。Z倍率不改变任何坐标或误差。可旋转、缩放、平移；<a href="report.html" target="_top">返回报告</a>。</p></main>','<script src="plotly-2.35.2.min.js"></script><script>'+viewer_js+'</script>'))
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
write('axis_fluctuations.html',page('20:14 稳定保护1.0｜XYZ方向误差波动','<main style="margin:0;padding:14px;max-width:none"><h2 style="margin:0">相对移动后综合示教圆的 XYZ 偏差</h2><p>测量点减去圆周最近点；无平滑、无剔除。各轴分量用于看波动，三维距离=√(ΔX²+ΔY²+ΔZ²)。图例可显示卡尔曼融合后视觉；拖选放大，双击复位。</p><div id="axes"></div></main>','<script src="plotly-2.35.2.min.js"></script><script>window.axisViewerReady=Plotly.newPlot("axes",'+json.dumps(axis_traces,ensure_ascii=False)+','+json.dumps(axis_layout)+',{responsive:true,displaylogo:false,scrollZoom:true});</script>'))
# Time windows reveal startup concentration without suppressing peaks.
windows=[]
for key,d in data.items():
    relative=(d.timestamp_ms-origin)/1000
    for name,mask in [('前60秒',relative<60),('60秒以后',relative>=60)]:
        if mask.any():windows.append(dict(trajectory=labels[key],window=name,**g.metric(d.distance_mm[mask]),above_3mm=int((d.distance_mm[mask]>3).sum())))
pd.DataFrame(windows).to_csv(OUT/'time_window_metrics.csv',index=False,encoding='utf-8-sig')
# Static overview with true coordinates; XZ remains equal scale.
im=Image.new('RGB',(1700,1150),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',23);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
draw.text((45,15),'20:14 稳定保护1.0 在线纠偏｜自动卡尔曼｜固定综合示教参考',fill='#203345',font=font)
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



config=json.loads((OUT/'run_config.json').read_text(encoding='utf-8'));local=r['local_registration'];sampling=r['image_sampling']
pair_configs=json.loads((OUT.parent/'20261021_stability_pair_comparison/run_configs.json').read_text(encoding='utf-8'))
comparison_rows=[];report_hashes={}
for cfg in pair_configs:
    path=OUT.parent/cfg['directory']/'metrics.csv';previous=pd.read_csv(path);report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for key in labels:
        a=previous[previous.trajectory.eq(key)].iloc[0]
        comparison_rows.append(dict(session=cfg['label'],name=labels[key],n=int(a.n),mean_mm=float(a.mean_mm),max_mm=float(a.max_mm),rmse_mm=float(a.rmse_mm),within3=f"{100*a.within_3mm_fraction:.2f}%",over3=int(a.above_3mm_count)))
pd.DataFrame(comparison_rows).to_csv(OUT/'stability_pair_comparison.csv',index=False,encoding='utf-8-sig')
comparison_table=table(comparison_rows,[('session','会话'),('name','轨迹'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('within3','≤3 mm'),('over3','>3 mm点数')])
local_rows=[]
for cfg in pair_configs:
    path=OUT.parent/cfg['directory']/'local_registration_summary.json';item=json.loads(path.read_text(encoding='utf-8'));report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    local_rows.append(dict(label=cfg['label'],**item))
local_compare=table(local_rows,[('label','会话'),('candidate_count','局部候选'),('accepted_count','候选通过'),('rejected_count','候选拒绝'),('writeback_record_count','写回记录'),('verified_success_count','后验成功'),('stability_evaluated_count','稳定收益评估'),('stability_reject_count','稳定保护拒绝')])
gain_table=table([dict(name=k,**v) for k,v in f['weight_stats'].items()],[('name','字段'),('n','有值点数'),('min','最小'),('median','中位'),('max','最大')])
local_df=pd.read_csv(OUT/'local_registration_metrics.csv');verified_df=pd.read_csv(OUT/'local_verification_records.csv');phase_df=pd.read_csv(OUT/'first_verified_update_windows.csv')
phase_table=table([dict(name=labels[a['trajectory']],phase='首次后验确认前' if a['phase']=='before_first_verified_update' else '首次后验确认后',**{k:a[k] for k in ['n','mean_mm','max_mm','rmse_mm','boundary_time']}) for a in phase_df.to_dict('records')],[('name','轨迹'),('phase','记录时段'),('boundary_time','分界时刻'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm')])
verified_table=table(verified_df.sort_values('time_ms').tail(8).to_dict('records'),[('time','后验记录时刻'),('succeeded','流程成功'),('independently_verified','新帧确认'),('cumulative_translation_mm','累计平移/mm'),('cumulative_rotation_deg','累计旋转/°'),('maximum_keypoint_displacement_mm','最大关键点位移/mm')])
image_df=pd.read_csv(OUT/'nearest_saved_peak_images.csv');image_table=table([dict(name=labels[a['trajectory']],**a) for a in image_df.to_dict('records')],[('name','峰值轨迹'),('peak_time','峰值时间'),('peak_error_mm','峰值/mm'),('image_time','最近已保存图像时间'),('time_offset_ms','图像减峰值/ms'),('frame_id','图像帧')])
evidence=OUT/'evidence';evidence.mkdir(exist_ok=True);gallery=[]
session=ROOT/'output/online_correction'/r['session']
for stem,group in image_df.groupby('stem',sort=False):
    row=group.iloc[0];caption=' / '.join(labels[x] for x in group.trajectory)
    for kind in ['source','enhanced']:
        original=session/'vision_images'/(stem+'_'+kind+'.png');copied=evidence/original.name;shutil.copy2(original,copied)
        assert original.read_bytes()==copied.read_bytes();report_hashes[str(original)]=hashlib.sha256(original.read_bytes()).hexdigest()
    gallery.append('<figure><figcaption>'+html.escape(caption+'；保存时间'+str(row.image_time)+'；它与各峰值的时间差见上表，非零时不是峰值同帧图。')+'</figcaption><div style="display:grid;grid-template-columns:1fr 1fr;gap:10px"><div><p>原始图像</p><img src="evidence/'+stem+'_source.png" alt="已保存原始图像"></div><div><p>增强图像</p><img src="evidence/'+stem+'_enhanced.png" alt="同帧已保存增强图像"></div></div></figure>')
gallery_html=''.join(gallery)
if config['prefix'].endswith('185548_548'):
    policy_text='<p>本组归档schema=2：90份候选中68份记录稳定保护实际评估，2份通过、66份被稳定保护拒绝；其余22份没有进入稳定评估。旧schema没有base_safety_accepted字段，因此报告仅将实际评估的68份标为“可由稳定评估推断原安全门通过”，其余不补造原安全状态。66份稳定否决的具体历史/收益原因可在候选表回查，不能把未评估时的默认历史数值当成“没有历史”。</p><p>旧版历史保护将历史冲突用于否决；归档显示仅2次候选进入写回并完成后验验证。这说明本组更新较少，不证明整圈更准确或更安全。</p>'
    lead='本组TCP平均0.738 mm、RMSE 0.847 mm、最大2.570 mm，全部有效点在3 mm内。原始KCF和融合后视觉最大均3.987 mm；自动卡尔曼使视觉RMSE由0.858降至0.849 mm，小幅改善。本组相对综合参考的平均误差和RMSE低于20:14稳定保护1.0组，但局部更新仅2次。'
else:
    policy_text='<p>本组归档schema=3、history_policy_version=2：97份候选明确记录base_safety_accepted，31份通过、66份拒绝。31份进入稳定收益评估并通过；这31份history_enforced均为false，其中30份记录历史区域冲突。历史仅作监测，不参与否决，原安全门仍先执行。未评估候选的历史默认值不作为结论。</p><p>历史冲突统计是对候选变换的预测，并且历史几何对应未被独立校核，不是“场景已经发生同等实测退化”。31次后验成功也只说明在线规定的新帧条件满足，不证明长期整圈精度。对比18:55组，本组更新显著增加，但相对综合示教的TCP误差与KCF峰值同时增大。</p>'
    lead='本组TCP平均0.829 mm、RMSE 0.954 mm、最大2.784 mm，全部有效点在3 mm内。自动卡尔曼使视觉RMSE由0.934降至0.928 mm，改善很小；融合后仍有最大6.439 mm的局部偏离。本组完成31次局部后验验证，多于18:55组的2次，但本次轨迹指标未随更新次数增加而降低。'
relative_change=100*(kcf['rmse_mm']-final['rmse_mm'])/kcf['rmse_mm']
sampling_note='旧采样仅覆盖启动及在线早段，不把后续未保存图像解释为无异常。' if sampling['image_diagnostic_schema']=='1' else 'schema=2按启动、在线早/中/晚和异常进度桶采样；桶号不是物理焊缝段，异常采样仍可能与峰值隔数秒。'
start=r['summary']['start_time'].split('T')[1][:8];end=r['summary']['end_time'].split('T')[1][:8]
gain=f['weight_stats']['kcf_weight_x']
body=f'''<main><span class="badge">2026-10-21 · {html.escape(config['label'])}＋自动卡尔曼 · 固定综合示教参考</span>
<h1>{html.escape(config['label'])}在线纠偏结果分析<br>移动后综合示教参考下的空间误差与局部更新证据</h1>
<p class="meta">会话 {html.escape(r['session'])}｜采集 {start}–{end}（北京时间）｜长度单位mm｜session_state={r['summary']['session_state']}</p>
<div class="actions"><a href="index.html">独立打开三维图</a><a href="axis_fluctuations.html">XYZ波动图</a><a href="metrics.csv">主统计表</a><a href="stability_pair_comparison.csv">两组对照</a><a href="results.json">完整分析数据</a></div>
<div class="cards"><div class="card"><span>TCP平均误差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大误差</span><b>{tcp['max_mm']:.3f} mm</b><span>{tcp['above_3mm_count']}点超过3 mm</span></div><div class="card"><span>融合后视觉RMSE</span><b>{final['rmse_mm']:.3f} mm</b><span>最大 {final['max_mm']:.3f} mm</span></div><div class="card"><span>局部新帧后验成功</span><b>{local['verified_success_count']} 次</b><span>{local['candidate_count']}份候选；非独立真值精度</span></div></div>
<div class="lead"><b>结论：</b>{lead} <b>候选通过、写回成功、在线后验合格和整圈几何贴合是不同证据，分别报告。</b></div>
<p class="note"><b>参考固定不变：</b>此前四组历史示教等权综合的原始圆，半径 <b>142.035339635 mm</b>。本轮只按<b>融合前KCF</b>拟合圆心与法线，刚体移动综合示教，保持半径/尺度，不用TCP或融合后视觉寻优。本组与另一组各自估计当前位置；两组的移动后参考均非独立实测焊缝真值。</p>
<h2>1. 数据范围与坐标</h2><p>读取用户指定会话，原始机器人状态{r['source_counts']['robot']:,}条，视觉诊断{r['source_counts']['diagnostics']:,}条。TCP按online_running、CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0筛选，保留{tcp['n']:,}点；视觉按online_running、result_valid=1筛选，融合前后各{kcf['n']:,}点。未平滑、未拟合替换或删除峰值，原始记录只读。两组均正常结束，未将启动/停止等待段混入主统计。</p>
<p>全部轨迹统一到<b>变位机末节随动坐标系（材料系）</b>：TCP经RBCALIB转固定站系，再逐点消除Ry(S1)Rz(S2)；视觉像素用同帧机器人姿态和当前相机标定投影，并加回工具TCP原点到末节的Z=231 mm偏置。若切到工具TCP系，所有圆心Z减231 mm，半径与误差不变。投影复算与日志一致不等于历史标定绝对准确。</p>
<h2>2. 四轨迹三维图与XYZ波动</h2><p>蓝：TCP；橙：融合前KCF；灰色虚线：移动前综合示教；绿：移动后综合示教；紫色融合后视觉可从图例打开。支持三维/XY/XZ/YZ、两组示教独立切换、Z显示倍率，默认1×等比例。倍率不改变坐标或误差；生成W点是既有综合参考采样，不是本次新增实测点。</p>
<iframe src="index.html" title="四轨迹交互三维图" style="height:1040px" loading="eager"></iframe>
<h3>ΔX、ΔY、ΔZ随时间变化</h3><p>每个点减去参考圆最近点，可查看局部轴向波动；它不是TCP与视觉的同步配对差，也不评价沿缝时序滞后。</p><iframe src="axis_fluctuations.html" title="XYZ波动图" style="height:990px" loading="lazy"></iframe>
<figure><img src="preview.png" alt="等比例空间投影、全量在线误差与累计分布"><figcaption>全部有效点保留。细微Z波动可在交互图中放大查看。</figcaption></figure>
<h2>3. 综合示教来源、移动与圆参数</h2>{source_table}<p>沿用此前的历史W几何去重、分组周向代表点拟合和等权综合结果，不因本轮结果改变来源或半径。移动只对齐圆心与法线，不缩放；绕法线相位对圆几何不可辨识，采用最小旋转。TCP、KCF和融合后视觉的拟合圆只用于描述，测量轨迹仍为原始点。</p>{circle_table}
<h2>4. 主误差结果</h2>{metric_table}<pre>c = 移动后综合圆心；n = 圆平面单位法线；R = 固定142.035339635 mm
h = (p-c)·n；ρ = ||(p-c)-h n||
e = √((ρ-R)²+h²)
平均 = mean(e)；最大 = max(e)；RMSE = √mean(e²)</pre>
<p>与此前报告同口径，采用点到<b>连续三维综合圆周</b>的解析最短距离，不混用原单组示教折线、最近采样点或行号配对距离。按记录样本加权，不按弧长/时间重加权。3 mm只用于描述偏离，不擅自设为逐点工艺合格线。</p>
<h2>5. 误差分量、时间窗口与峰值</h2>{component_table}<p>有符号均值可能正负抵消，整体仍使用非负三维距离和RMSE。参考位置由KCF估计，可吸收整体位姿偏差，分量不能直接当独立标定误差。</p>{window_table}
<p>前60秒从保留轨迹的最早时间起算，不是会话启动时刻。TCP最大值在{tcp['max_time']}；融合前KCF最大在{kcf['max_time']}；融合后视觉最大在{final['max_time']}。事件按相邻记录均超3 mm归组，相邻时间间隔大于200 ms拆分，下表最多15段，完整列表保留。</p>{event_table}
<p>最大局部偏离为{labels[peak['trajectory']]}在{peak['max_time']}的{peak['max_mm']:.3f} mm，径向{peak['radial_at_peak_mm']:.3f} mm、离面{peak['plane_at_peak_mm']:.3f} mm。不凭三维轨迹单独判定是图像误跟踪、真实焊缝局部形状、位姿同步还是其它因素造成。</p>
<h2>6. 自动卡尔曼同帧影响</h2><p>{f['applied_count']:,}/{kcf['n']:,}个有效视觉点记录融合应用，未融合{kcf['n']-f['applied_count']}点；模板重新初始化标记{f['template_reinitialized_count']}次。KCF X/Y增益中位约{gain['median']*100:.2f}%，范围{gain['min']*100:.2f}%～{gain['max']*100:.2f}%，不是固定0.1/0.9。记录增益、归一化响应和观测噪声如下：</p>{gain_table}
<p>融合前后空间位移平均{f['movement_mm']['mean_mm']:.3f} mm、P95 {f['movement_mm']['p95_mm']:.3f} mm、最大{f['movement_mm']['max_mm']:.3f} mm。固定同一参考，视觉RMSE从{kcf['rmse_mm']:.3f}降至{final['rmse_mm']:.3f} mm（约{relative_change:.2f}%）；超3 mm点数从{kcf['above_3mm_count']}变为{final['above_3mm_count']}。误差变小{f['improved_count']:,}点、变大{f['worsened_count']:,}点、近似不变{f['unchanged_count']:,}点，判断容差1e-10 mm。平均指标改善不代表每帧更好，也不证明仿真先验绝对准确。correlation/fusion jump rejected分别为{f['correlation_jump_rejected']}/{f['fusion_jump_rejected']}。</p>
<h2>7. 局部表面ICP、策略与后验</h2>{local_compare}{policy_text}
<p>本组{local['writeback_record_count']}份写回、{local['verification_record_count']}份后验记录，其中{local['verified_success_count']}份succeeded且verified_by_independent_observation，rollback_attempted记录{local['rollback_attempted_count']}份。“独立观测”是在线链路术语，仍使用相机/标定/KCF，不能升级为外部独立真值。以下为最多8份较晚后验记录；累计平移/旋转是归档的事务预算量，不是实测工件误差。</p>{verified_table}
<p>按首次后验确认时刻划分有效记录如下。前段样本少且处于起始过渡，后段覆盖不同位置与时间；该分段只能描述，不能把前后差异全部归因于一次更新。</p>{phase_table}
<p>本组局部冻结窗口有{local['raw_archive_replay_count']}个不同帧的融合前独立观测可重放，和诊断像素投影的最大差{local['raw_archive_replay_max_mm']:.2e} mm。另有<b>轨迹预测ICP</b> {arc['batch_count']}批、{arc['applied']}批应用，与局部工件位姿更新不同；轨迹已释放点到同批原始待处理折线最大偏离{arc['released_departure_max_mm']:.3f} mm，不能当成工件位移或本报告的TCP误差。</p>
<h2>8. 已保存图像与峰值的关系</h2><p>本组保存{sampling['pairs']}对图像，覆盖{sampling['first_saved_time']}–{sampling['last_saved_time']}。{sampling_note} 下表逐一标明最近已保存图像与各轨迹峰值的时间差；TCP时间是机器人记录时间，图像时间来自相机帧，时间接近也不等于同一物理观测。</p>{image_table}{gallery_html}
<p>仅当时间差为0且视觉帧对应时才可称为该视觉峰值同帧图。其它图片只提供邻近情形，不能据它证明峰值原因；未保存图像的时段保留证据缺口。图像采样桶与局部ICP历史区域是不同概念。</p>
<h2>9. 两组同口径对照</h2>{comparison_table}<p>18:55组TCP平均/RMSE为0.738/0.847 mm，20:14组为0.829/0.954 mm；两组TCP均无点超3 mm。后者融合前KCF峰值6.574 mm高于前者3.987 mm。局部后验成功从2次增到31次，与轨迹误差未降低同时存在，不能按更新次数给精度排名。两轮各自用原始KCF定位同一综合参考，采集与状态不同，非只改变一个参数的受控实验。</p>
<h2>10. 敏感性、验证与后续建议</h2><p>按交错20°扇区留出，KCF合并RMSE {r['holdout']['pooled']['rmse_mm']:.3f} mm；有效KCF周向覆盖约{r['coverage_deg']:.2f}°。留出减少同点拟合/评价复用，但共享传感器与标定偏差仍存在。历史示教留一组半径敏感性如下，主半径始终固定，不从表中挑选更好结果。</p>{sens_table}
<p>固定站系投影和末节随动系日志最大复算差分别{r['verification']['final_projection_max_mm']:.2e}、{r['verification']['positioner_replay_max_mm']:.2e} mm。输入哈希、逐点分量、独立标量几何、同帧融合、局部归档与图表控件分别校验。使用当前可读标定复算，缺乏每次运行完整的标定/可执行版本快照；源码/文档只作语义解释，运行结论优先按本组归档字段与schema判断。</p>
<p>优先核对本组峰值附近图像、检测与机器人状态，并区分当前焊点收益、历史监测预测与真实写回后的新帧观测。若要判定稳定策略带来的改善，应在相近初始工件位置与运行条件下重复测试，同时关注TCP平均/RMSE/峰值、检测峰值和局部后验。当前没有移动后实物焊缝的独立测量，报告不承诺绝对精度或长期整圈稳定。</p>
<div class="actions"><a href="TCP_circle_error_components.csv">TCP逐点误差</a><a href="raw_KCF_circle_error_components.csv">原始KCF误差</a><a href="final_vision_circle_error_components.csv">融合后误差</a><a href="fusion_effects.csv">同帧融合</a><a href="over_3mm_events.csv">峰值事件</a><a href="local_registration_metrics.csv">局部候选</a><a href="local_verification_records.csv">后验记录</a><a href="local_writeback_records.json">写回摘要</a><a href="trajectory_icp_effects.csv">轨迹ICP</a><a href="nearest_saved_peak_images.csv">峰值图片索引</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">数值复核</a><a href="browser_verification.json">浏览器检查</a></div>
<p>复现依次运行本目录analyze.py、build_report.py、verify.py。HTML与Plotly依赖可离线使用。所有新增文件只在本分析目录；原始记录与先前分析均只读。浏览器使用无鼠标渲染及DOM事件检查，手动拖拽/缩放/平移未验收。</p>
<footer>展示与协作方法沿用此前读取的<a href="prior_notion_workflow_snapshot.json">Notion工作流快照</a>（2026-10-19/18），不声称已核对在线最新版。按用户要求不使用鼠标、不删除文件；参考固定为此前综合原始示教，移动后对比。</footer></main>'''
write('report.html',page(config['label']+'＋自动卡尔曼｜移动后综合示教误差分析',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2))
print(json.dumps(dict(report=str(OUT/'report.html'),local_verified=local['verified_success_count'],rmse_reduction_percent=relative_change),ensure_ascii=False,indent=2))
