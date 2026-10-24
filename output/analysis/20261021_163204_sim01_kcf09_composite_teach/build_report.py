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
labels={'TCP':'TCP 实际轨迹','raw_KCF':'KCF 融合前','final_vision':'融合后视觉'}
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
write('index.html',page('16:32｜综合示教参考四轨迹三维图','<main style="max-width:none;margin:0;padding:16px"><h2 style="margin:0">16:32｜仿真0.1 / KCF0.9 · 四轨迹空间对照</h2><p>变位机末节随动系，单位mm。灰色/绿色均为此前综合示教参考，半径142.035340 mm；生成W点不是新实测点。蓝色TCP、橙色融合前KCF保留全部有效点；紫色融合后视觉可点击图例打开。移动后参考为事后估计，不能当独立真值。</p><p>'+controls+'</p><div id="plot" style="height:730px"></div><p class="meta">显示切换不改变误差定义：统计始终使用连续综合圆周。Z倍率不改变任何坐标或误差。可旋转、缩放、平移；<a href="report.html" target="_top">返回报告</a>。</p></main>','<script src="plotly-2.35.2.min.js"></script><script>'+viewer_js+'</script>'))
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
write('axis_fluctuations.html',page('16:32｜XYZ方向误差波动','<main style="margin:0;padding:14px;max-width:none"><h2 style="margin:0">相对移动后综合示教圆的 XYZ 偏差</h2><p>测量点减去圆周最近点；无平滑、无剔除。各轴分量用于看波动，三维距离=√(ΔX²+ΔY²+ΔZ²)。图例可显示融合后视觉；拖选放大，双击复位。</p><div id="axes"></div></main>','<script src="plotly-2.35.2.min.js"></script><script>window.axisViewerReady=Plotly.newPlot("axes",'+json.dumps(axis_traces,ensure_ascii=False)+','+json.dumps(axis_layout)+',{responsive:true,displaylogo:false,scrollZoom:true});</script>'))
# Time windows reveal startup concentration without suppressing peaks.
windows=[]
for key,d in data.items():
    relative=(d.timestamp_ms-origin)/1000
    for name,mask in [('前60秒',relative<60),('60秒以后',relative>=60)]:
        if mask.any():windows.append(dict(trajectory=labels[key],window=name,**g.metric(d.distance_mm[mask]),above_3mm=int((d.distance_mm[mask]>3).sum())))
pd.DataFrame(windows).to_csv(OUT/'time_window_metrics.csv',index=False,encoding='utf-8-sig')
# Static overview with true coordinates; XZ remains equal scale.
im=Image.new('RGB',(1700,1150),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',23);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
draw.text((45,15),'16:32 在线纠偏｜仿真0.1 / KCF0.9｜固定综合示教参考',fill='#203345',font=font)
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
body=f'''<main><span class="badge">2026-10-21 · 16:32新会话 · 固定历史综合示教参考</span>
<h1>在线纠偏结果分析：仿真0.1 / KCF0.9<br>综合示教参考下的空间误差、轴向波动与融合效果</h1>
<p class="meta">会话 {html.escape(r['session'])}｜采集 16:32:04–16:42:14（北京时间）｜出风口正20 mm｜所有长度单位mm</p>
<div class="actions"><a href="index.html">独立打开三维图</a><a href="axis_fluctuations.html">XYZ波动图</a><a href="metrics.csv">主统计表</a><a href="results.json">完整分析数据</a></div>
<div class="cards"><div class="card"><span>TCP平均误差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>KCF融合前平均误差</span><b>{kcf['mean_mm']:.3f} mm</b><span>RMSE {kcf['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大偏离</span><b>{tcp['max_mm']:.3f} mm</b><span>{tcp['max_time']}；{tcp['above_3mm_count']}点超过3 mm</span></div><div class="card"><span>KCF最大偏离</span><b>{kcf['max_mm']:.3f} mm</b><span>{kcf['max_time']}；{kcf['above_3mm_count']}点超过3 mm</span></div></div>
<div class="lead"><b>结论：</b>本轮TCP与融合前KCF的平均几何距离都在0.9 mm以内，P95约1.5～1.6 mm。相较15:23自动卡尔曼那轮，KCF最大偏离由6.471降至3.419 mm，TCP最大偏离则由2.569升至3.837 mm，TCP RMSE也由0.895升至0.985 mm。<b>检测轨迹峰值变小，但TCP局部偏离仍需关注，不能据此认定整体控制效果提升。</b></div>
<p class="note"><b>参考对象已按用户确认固定：</b>此前四组历史示教等权综合的原始示教圆，半径 <b>142.035339635 mm</b>。本轮只按融合前KCF估计圆心和法线进行刚体移动，未重估综合半径，未使用TCP优化参考。该移动后对象是估计参考，不能直接称为独立真实焊缝真值。</p>
<h2>1. 数据范围、权重与坐标</h2><p>原始机器人状态{r['source_counts']['robot']:,}条，视觉诊断{r['source_counts']['diagnostics']:,}条。TCP筛选 online_running、CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0，保留{tcp['n']:,}点；KCF及融合后视觉筛选 online_running、result_valid=1，各保留{kcf['n']:,}点。无测量点平滑、拟合替换或峰值剔除；拟合只用于参考定位和描述圆参数。</p>
<p>全部有效在线视觉点记录的KCF X/Y权重均为0.9，仿真先验权重均为0.1，{f['applied_count']:,}点记录fusion_applied=1；像素融合公式复算最大差{f['formula_max_abs_px']:.8f} px。会话目录名称和实际日志一致；固定比例直接作用于当前KCF与仿真像素，不应称作自动卡尔曼权重。</p>
<p>统一坐标为<b>变位机末节随动坐标系（材料系）</b>：机器人基坐标经RBCALIB转换到固定站系，再逐点消除Ry(S1)Rz(S2)外轴旋转。视觉通过当前相机标定和该帧机器人姿态投影，复算与归档一致。与变位机工具TCP系相比，本报告的Z加回231 mm；如切到工具TCP系，所有圆心Z减231 mm，半径和误差不变。</p>
<h2>2. 四轨迹三维图与各轴波动</h2><p>蓝：TCP；橙：融合前KCF；灰色虚线：移动前综合原始示教；绿：移动后综合示教。紫色融合后视觉可从图例打开。提供3D、XY、XZ、YZ视角、两组示教独立显示切换以及Z倍率；默认1×等比例。生成W点是上轮综合后的参考采样，并非独立实测焊缝点。</p>
<iframe src="index.html" title="四轨迹交互三维图" style="height:1040px" loading="eager"></iframe>
<h3>相对参考圆的ΔX、ΔY、ΔZ</h3><p>每个点减去圆周最近点，既能观察各轴波动，也避免整圈运动的百毫米坐标变化掩盖毫米量级误差。不是TCP与视觉按同一时间配对的差值；各轴分量不能直接当作沿缝时滞。</p>
<iframe src="axis_fluctuations.html" title="XYZ波动图" style="height:990px" loading="lazy"></iframe>
<figure><img src="preview.png" alt="XY与XZ等比例投影、三维距离及累计分布"><figcaption>静态总览；XZ保持等比例，微小Z波动请使用交互图显示倍率和各轴偏差图。</figcaption></figure>
<h2>3. 参考来源与圆参数</h2>{source_table}<p>沿用原综合结果：各组历史W几何去重、2°周向分箱取代表点、分别拟合平面圆，再等权综合半径。历史来源未重新挑选，不按本轮误差反向选择最有利半径。原始综合示教中心/法线与上一轮相同，移动后中心/法线对齐本轮融合前KCF。绕法线的相位对圆几何不可辨识，采用最小旋转。</p>{circle_table}
<p>上述所有圆心均在同一末节随动系。TCP、KCF及融合后圆仅用于描述，不替换原始测量走势。原始综合与移动后半径相同；移动后圆心、法线等于KCF拟合结果，是位姿对齐的定义。</p>
<h2>4. 主误差结果</h2>{metric_table}<pre>c = 移动后综合圆心；n = 圆平面单位法线；R = 142.035339635 mm
h = (p-c)·n；ρ = ||(p-c)-h n||
三维误差 e = √((ρ-R)²+h²)
平均 = mean(e)；最大 = max(e)；RMSE = √mean(e²)</pre><p>误差是到<b>连续三维综合圆周</b>的解析最短距离。与此前最终综合报告同口径，不使用圆周绘图折线、最近采样点或按行号配对误差。统计按记录样本加权，保留全部有效在线点，未使用时序加权或弧长加权。不把参考来源于KCF的结果解释成绝对标定精度。</p>
<h2>5. 局部峰值、各轴分量与时间分布</h2>{component_table}<p>径向/离面均值有正负，三维距离非负。当前参考中心和法线由KCF定位，会吸收整体位姿偏差，各轴误差不能直接当成独立标定误差。</p>{window_table}
<p><b>TCP超3 mm的49个点全部集中在有效在线数据开始后的前60秒，60秒以后最大误差2.899 mm。</b>以上时间从保留轨迹的最早时间起算，不是会话启动时间。</p><p>以下事件按相邻记录误差均超过3 mm归组，记录间隔大于200 ms则拆分；3 mm用于观察偏离，不宣称为严格工艺合格线。表内保留TCP和融合前后视觉事件，最多显示15段，完整结果可下载。</p>{event_table}<p>本轮最大偏离为{labels[peak['trajectory']]}在{peak['max_time']}的{peak['max_mm']:.3f} mm，径向分量{peak['radial_at_peak_mm']:.3f} mm、离面分量{peak['plane_at_peak_mm']:.3f} mm。仅凭轨迹尚不能区分控制滞后、图像误差或真实局部形状；未将其直接归因为某个模块故障。</p>
<h2>6. 固定0.1/0.9融合的实际影响</h2><p>有效视觉点的融合前后空间位移平均{f['movement_mm']['mean_mm']:.3f} mm，P95 {f['movement_mm']['p95_mm']:.3f} mm，最大{f['movement_mm']['max_mm']:.3f} mm。固定同一个移动后参考，融合前RMSE {kcf['rmse_mm']:.3f} mm，融合后{final['rmse_mm']:.3f} mm，增加{f['geometry_rmse_change_mm']:.3f} mm；融合前后最大分别为{kcf['max_mm']:.3f} / {final['max_mm']:.3f} mm，超3 mm点数分别为{kcf['above_3mm_count']} / {final['above_3mm_count']}。</p><p><b>这轮数据未显示固定融合能降低该几何指标。</b>但参考位姿由融合前KCF建立，对它存在共享观测优势；融合后几何距离略大也不能单独证明仿真先验无效或在线控制变差。记录中correlation_jump_rejected={f['correlation_jump_rejected']}、fusion_jump_rejected={f['fusion_jump_rejected']}。</p>
<h2>7. 在线ICP链路证据</h2><p>对应16:32会话的局部配准目录有{arc['candidates']}个候选归档，其中{arc['accepted']}个记录验收通过，{arc['writebacks']}份写回记录，{arc['verified']}份独立后验观测验证成功。此处的“独立”是在线链路的后验观测标记，仍可能共享相机、标定和检测偏差，不能升级成外部测量真值。</p><p>已将归档里{r['verification']['raw_archive_replay_count']}个不同帧的融合前独立观测与诊断像素复算对照，最大差{r['verification']['raw_archive_replay_max_mm']:.2e} mm。候选验收/拒绝、前后残差保留在<a href="local_registration_metrics.csv">配准指标表</a>；写回和后验清单见<a href="writeback_inventory.json">归档摘要</a>。这些证据支持链路实际运行过，不直接等于相对实物焊缝的绝对精度。</p>
<h2>8. 与15:23上一轮同参考定义对比</h2>{comparison_table}<p>两轮沿用同一历史综合半径，各自由本轮融合前KCF估计参考位姿。比较的是相同几何评价方式下的描述结果；采集批次、机器人状态和检测观测不同，未构成只改变一个融合参数的受控实验。尤其TCP最大值和RMSE升高，与KCF峰值降低同时存在，应分别报告。</p>
<h2>9. 参考敏感性与结论边界</h2><p>按交错20°扇区留出，一半扇区估计圆心/法线，另一半评价，KCF合并留出RMSE为{r['holdout']['pooled']['rmse_mm']:.3f} mm；本轮有效KCF周向覆盖约{r['coverage_deg']:.2f}°。留出验证减少同一点同时拟合与评价的复用，但不能消除共享系统误差。</p>{sens_table}<p>表中只展示历史综合来源“每次留掉一组”时的尺寸敏感性，<b>主结果的半径始终固定，不从该表选择更有利参考</b>。未提供本轮工件移动后的独立实测焊缝；事后整圈参考也不是在线控制器每一帧的实际目标。当前只复核现有标定文件，没有完整历史标定快照；复算一致说明当前转换可解释日志，不能代替历史标定准确性的证明。</p>
<h2>10. 后续建议与复现材料</h2><p>优先回查16:33:07附近TCP偏离的状态时序、纠偏指令与图像，再核对16:33:33附近KCF峰值，保留原始点。若要判断0.1/0.9是否优于自动卡尔曼，建议在相同参考、相近运行条件下重复采集，并同时比较TCP峰值、RMSE、离面波动及异常段；判断绝对精度则需要独立测得的移动后焊缝或工件位姿。</p>
<div class="actions"><a href="TCP_circle_error_components.csv">TCP逐点误差</a><a href="raw_KCF_circle_error_components.csv">KCF逐点误差</a><a href="final_vision_circle_error_components.csv">融合后逐点误差</a><a href="over_3mm_events.csv">峰值事件</a><a href="fusion_effects.csv">融合前后配对</a><a href="axis_residual_summary.csv">各轴统计</a><a href="previous_run_comparison.csv">两轮对比</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">验证记录</a></div>
<p>验证记录：181项输入哈希、95个独立标量几何复算及逐点误差分量恒等式通过；无鼠标Edge浏览器渲染和DOM事件检查覆盖视角、两组示教独立切换及Z倍率，手动拖拽/缩放/平移未执行。<a href="browser_verification.json">浏览器检查记录</a>。</p><p>复现：依次运行本目录analyze.py、build_report.py、verify.py。原始输入只读，所有新增产物均位于本目录；依赖沿用已有NumPy、pandas、Pillow与本地Plotly，不需要联网加载图表。</p>
<footer>工作流参考：用户提供的<a href="https://fancy-margin-a38.notion.site/AI-3f7aa04fa8a981798bdaeec496f904f1?pvs=74">Notion AI协作指南</a>。公开页本次读取失败；实际采用先前保存的<a href="prior_notion_workflow_snapshot.json">数据分析展示偏好与协作工作流正文快照</a>（读取时间2026-10-19/18），不声称已核对当前在线最新版。当前用户指令优先：不使用鼠标、不删除文件，参考固定为此前综合后的原始示教。</footer></main>'''
write('report.html',page('在线纠偏｜仿真0.1 KCF0.9｜综合示教参考误差分析',body))
print(json.dumps(dict(report=str(OUT/'report.html'),windows=windows),ensure_ascii=False,indent=2))
