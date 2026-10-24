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
write('index.html',page('10月20日20:22｜综合示教参考四轨迹三维图','<main style="max-width:none;margin:0;padding:16px"><h2 style="margin:0">10月20日20:22｜自动卡尔曼 · 四轨迹空间对照</h2><p>变位机末节随动系，单位mm。灰色/绿色均为此前综合示教参考，半径142.035340 mm；生成W点不是新实测点。蓝色TCP、橙色原始KCF保留全部有效点；紫色卡尔曼融合后视觉可点击图例打开。移动后参考为事后估计，不能当独立真值。</p><p>'+controls+'</p><div id="plot" style="height:730px"></div><p class="meta">显示切换不改变误差定义：统计始终使用连续综合圆周。Z倍率不改变任何坐标或误差。可旋转、缩放、平移；<a href="report.html" target="_top">返回报告</a>。</p></main>','<script src="plotly-2.35.2.min.js"></script><script>'+viewer_js+'</script>'))
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
write('axis_fluctuations.html',page('10月20日20:22｜XYZ方向误差波动','<main style="margin:0;padding:14px;max-width:none"><h2 style="margin:0">相对移动后综合示教圆的 XYZ 偏差</h2><p>测量点减去圆周最近点；无平滑、无剔除。各轴分量用于看波动，三维距离=√(ΔX²+ΔY²+ΔZ²)。图例可显示卡尔曼融合后视觉；拖选放大，双击复位。</p><div id="axes"></div></main>','<script src="plotly-2.35.2.min.js"></script><script>window.axisViewerReady=Plotly.newPlot("axes",'+json.dumps(axis_traces,ensure_ascii=False)+','+json.dumps(axis_layout)+',{responsive:true,displaylogo:false,scrollZoom:true});</script>'))
# Time windows reveal startup concentration without suppressing peaks.
windows=[]
for key,d in data.items():
    relative=(d.timestamp_ms-origin)/1000
    for name,mask in [('前60秒',relative<60),('60秒以后',relative>=60)]:
        if mask.any():windows.append(dict(trajectory=labels[key],window=name,**g.metric(d.distance_mm[mask]),above_3mm=int((d.distance_mm[mask]>3).sum())))
pd.DataFrame(windows).to_csv(OUT/'time_window_metrics.csv',index=False,encoding='utf-8-sig')
# Static overview with true coordinates; XZ remains equal scale.
im=Image.new('RGB',(1700,1150),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',23);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
draw.text((45,15),'10月20日20:22 在线纠偏｜自动卡尔曼｜固定综合示教参考',fill='#203345',font=font)
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


comparison_rows=[];report_hashes={}
for session,label in [('20261020_202202_auto_kalman_composite_teach','10/20 20:22 自动卡尔曼'),('20261020_204102_kcf_only_composite_teach','10/20 20:41 仅KCF'),('20261021_positive20_teach_synthesis','10/21 15:23 自动卡尔曼'),('20261021_163204_sim01_kcf09_composite_teach','10/21 16:32 固定0.1/0.9')]:
    path=OUT.parent/session/'metrics.csv';previous=pd.read_csv(path);report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for key in labels:
        a=previous[previous.trajectory.eq(key)].iloc[0]
        comparison_rows.append(dict(session=label,name=labels[key],n=int(a.n),mean_mm=float(a.mean_mm),max_mm=float(a.max_mm),rmse_mm=float(a.rmse_mm),within3=f"{float(a.within_3mm_fraction)*100:.2f}%"))
pd.DataFrame(comparison_rows).to_csv(OUT/'four_run_comparison.csv',index=False,encoding='utf-8-sig')
comparison_table=table(comparison_rows,[('session','会话'),('name','轨迹'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('within3','≤3 mm')])
gains=f['weight_stats'];gain_rows=[dict(name=key,**value) for key,value in gains.items()]
gain_table=table(gain_rows,[('name','记录字段'),('n','有值点数'),('min','最小'),('median','中位数'),('max','最大')])
relative_improvement=100*(kcf['rmse_mm']-final['rmse_mm'])/kcf['rmse_mm']
body=f'''<main><span class="badge">2026-10-20 · 20:22会话 · 自动卡尔曼 · 固定综合示教参考</span>
<h1>在线纠偏结果分析：自动卡尔曼<br>移动后综合示教参考下的轨迹误差与融合效果</h1>
<p class="meta">会话 {html.escape(r['session'])}｜采集 2026-10-20 20:22:02–20:34:26（北京时间）｜出风口正20 mm｜长度单位mm</p>
<div class="actions"><a href="index.html">独立打开三维图</a><a href="axis_fluctuations.html">XYZ波动图</a><a href="metrics.csv">主统计表</a><a href="results.json">完整分析数据</a></div>
<div class="cards"><div class="card"><span>TCP平均误差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大误差</span><b>{tcp['max_mm']:.3f} mm</b><span>全部有效点在3 mm内</span></div><div class="card"><span>原始KCF / 融合后 RMSE</span><b>{kcf['rmse_mm']:.3f} / {final['rmse_mm']:.3f}</b><span>融合后下降{relative_improvement:.2f}%</span></div><div class="card"><span>融合后视觉最大误差</span><b>{final['max_mm']:.3f} mm</b><span>{final['above_3mm_count']}点超过3 mm</span></div></div>
<div class="lead"><b>结论：</b>本组TCP平均误差{tcp['mean_mm']:.3f} mm、RMSE {tcp['rmse_mm']:.3f} mm、最大{tcp['max_mm']:.3f} mm，全部有效点在3 mm内。固定同一移动后综合参考，自动卡尔曼使视觉RMSE由{kcf['rmse_mm']:.3f}降至{final['rmse_mm']:.3f} mm，下降{relative_improvement:.2f}%，属于小幅改善；最大偏离仍超过6 mm，不能理解为消除了所有检测峰值。相较20:41仅KCF那组，TCP的平均、RMSE和最大值均较低，但这不是单参数受控实验。</div>
<p class="note"><b>参考保持不变：</b>此前四组历史示教等权综合的原始示教圆，半径固定 <b>142.035339635 mm</b>。本组只用<b>融合前原始KCF</b>估计圆心和法线，再刚体移动综合示教。TCP和融合后视觉均未用于调整参考，未按本组误差重选半径；移动后是事后估计参考，不是独立实测焊缝真值。</p>
<h2>1. 输入记录与实际融合模式</h2><p>只读取用户指定20:22会话。机器人状态{r['source_counts']['robot']:,}条、视觉诊断{r['source_counts']['diagnostics']:,}条；TCP筛选online_running、CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0，保留{tcp['n']:,}点。视觉筛选online_running、result_valid=1，原始和融合后各{kcf['n']:,}点。无测量点平滑、拟合替换或峰值剔除。</p>
<p>{f['applied_count']:,}个有效在线视觉点记录fusion_applied=1，另外{kcf['n']-f['applied_count']}点未融合。KCF增益有值{f['weights_available_count']:,}点，X/Y中位数均约{gains['kcf_weight_x']['median']*100:.2f}%，范围{gains['kcf_weight_x']['min']*100:.2f}%～{gains['kcf_weight_x']['max']*100:.2f}%；这是动态增益，不能套用固定0.1/0.9。模板重新初始化与Kalman seeded标记各{f['template_reinitialized_count']}次，计数不单独证明重初始化原因。</p>{gain_table}
<p>目录标注“启动在线ICP但不成功”作为用户给定的局部表面点云定位背景保留。本会话另有{arc['batch_count']}批轨迹预测ICP记录、{arc['applied']}批applied；项目当前文档与代码将其与修改工件位姿的局部表面ICP分开，因此不能将前者当作后者成功的证据。本次不重新判定局部表面ICP失败原因，也不凭轨迹批次推断发生了工件SET_POSE写回。</p>
<h2>2. 四轨迹三维图与各轴波动</h2><p>统一到<b>变位机末节随动坐标系（材料系）</b>，逐点消除S1/S2外轴旋转；视觉恢复到与示教相同的末节原点，较工具TCP系Z加回231 mm。若切到工具TCP系，所有圆心Z减231 mm即可，半径和距离不变。</p>
<p>蓝：TCP；橙：原始KCF；灰色虚线：移动前综合示教；绿：移动后综合示教参考。紫色卡尔曼融合后视觉可从图例打开。支持三维/XY/XZ/YZ视角、两组示教独立切换和Z显示倍率，默认1×等比例；显示倍率不改变数据。生成W点是综合参考采样，不是本次独立测量点。</p>
<iframe src="index.html" title="四轨迹交互三维图" style="height:1040px" loading="eager"></iframe>
<h3>XYZ方向偏差</h3><p>ΔX、ΔY、ΔZ为每个测量点减去参考圆最近点，用于查看局部波动；不是TCP与KCF同步配对差值，也不评价沿缝时序滞后。可在图例打开融合后曲线，检查原始KCF与卡尔曼结果的区别。</p>
<iframe src="axis_fluctuations.html" title="XYZ波动图" style="height:990px" loading="lazy"></iframe>
<figure><img src="preview.png" alt="四轨迹等比例投影、三维距离及累计分布"><figcaption>静态总览保留真实坐标与全部有效点；细小Z变化可使用交互倍率或各轴图观察。</figcaption></figure>
<h2>3. 综合示教来源与同坐标系圆参数</h2>{source_table}<p>沿用四种历史W几何的既有综合结果：按XYZ去重，各组周向分箱后分别拟合圆，等权综合半径。移动前圆心和法线不变；移动后对齐本组原始KCF，保持半径和尺度。圆的绕法线相位不可辨识，采用最小旋转。TCP、原始KCF及融合后圆只用于描述，不替换它们的原始走势。</p>{circle_table}
<h2>4. 主误差结果</h2>{metric_table}<pre>c = 移动后综合圆心；n = 单位法线；R = 固定综合半径142.035339635 mm
h = (p-c)·n；ρ = ||(p-c)-h n||
e = √((ρ-R)²+h²)
平均 = mean(e)；最大 = max(e)；RMSE = √mean(e²)</pre>
<p>与此前综合报告同口径，均为点到<b>连续三维综合圆周</b>的解析最短距离，不混用原单组示教折线、最近采样点或行号配对距离。各组按自己的记录样本加权，不做弧长/时间重加权；显示切换不改变统计。3 mm用于观察偏离，不擅自定义为工艺合格线。</p>
<h2>5. 各轴、径向/离面与局部峰值</h2>{component_table}<p>径向、离面均值有正负，三维距离非负。参考位姿由KCF确定，会吸收整体位置和倾角偏差，分量不能直接解释为独立标定误差。</p>{window_table}
<p>TCP无点超过3 mm；原始KCF有{kcf['above_3mm_count']}点，融合后有{final['above_3mm_count']}点超过3 mm。事件按相邻记录均超3 mm归组，时间间隔大于200 ms再拆分；下表保留原始与融合后事件，最多15段。起始段和峰值均未删除。</p>{event_table}
<p>全部轨迹中的最大偏离是{labels[peak['trajectory']]}在{peak['max_time']}的{peak['max_mm']:.3f} mm，径向{peak['radial_at_peak_mm']:.3f} mm、离面{peak['plane_at_peak_mm']:.3f} mm。原始KCF与融合后各自的最大值发生在不同帧，不能把两个最大值相减当成同一帧的改善。</p>
<h2>6. 卡尔曼融合的同帧影响</h2><p>原始与融合后共有{kcf['n']:,}对同帧观测，三维位置变化平均{f['movement_mm']['mean_mm']:.3f} mm、P95 {f['movement_mm']['p95_mm']:.3f} mm、最大{f['movement_mm']['max_mm']:.3f} mm。相对固定参考，融合后误差变小{f['improved_count']:,}点、变大{f['worsened_count']:,}点、近似不变{f['unchanged_count']:,}点（判断容差1e-10 mm）。</p>
<p>视觉RMSE下降{kcf['rmse_mm']-final['rmse_mm']:.4f} mm，平均距离由{kcf['mean_mm']:.3f}变为{final['mean_mm']:.3f} mm，超3 mm点数由{kcf['above_3mm_count']}变为{final['above_3mm_count']}。这说明本组对该几何指标有小幅改善，<b>没有消除所有峰值，也不保证每帧都更接近参考</b>。参考位姿使用融合前KCF，会给它共享观测优势；这些距离无法单独评估仿真先验的独立准确性。逐帧增益、位移及误差变化见<a href="fusion_effects.csv">融合前后配对表</a>。</p>
<h2>7. 轨迹ICP记录与四组对照</h2><p>轨迹预测ICP共{arc['batch_count']}批，均enabled=1、attempted=1，{arc['applied']}批应用、{arc['not_applied']}批未应用。已应用批次released_target到同批raw_pending折线的最大偏离{arc['released_departure_max_mm']:.3f} mm，各批平均偏离中位数{arc['batch_mean_departure_median_mm']:.3f} mm。该量不是TCP到焊缝的误差，也不是工件位姿位移或局部表面配准残差，不能将矩阵平移列直接解释为实物移动。</p>{comparison_table}
<p>四组使用同一综合半径，各自由本组原始KCF定位参考。20:22自动卡尔曼组的TCP平均、RMSE和最大值均低于20:41仅KCF组；与次日15:23组相比，TCP平均/RMSE稍高、最大值稍低。它们不是只改变融合参数的受控重复，采集时段、观测和状态均可能不同，不将全部差异归因于卡尔曼或局部ICP开关。</p>
<h2>8. 参考敏感性、验证和后续建议</h2><p>按交错20°扇区留出，一半估计圆心/法线、另一半评价，KCF合并RMSE为{r['holdout']['pooled']['rmse_mm']:.3f} mm；有效KCF周向覆盖约{r['coverage_deg']:.2f}°。留出减少重复使用同一点，但仍共享传感器与标定偏差。历史示教留一组的半径敏感性如下，主参考不按这些结果重新选择。</p>{sens_table}
<p>投影到固定站系的日志最大复算差{r['verification']['final_projection_max_mm']:.2e} mm，末节随动系最大复算差{r['verification']['positioner_replay_max_mm']:.2e} mm。当前标定能解释记录不代表历史标定无误；没有移动后焊缝的独立实测，所以所有结论均为相对该综合估计参考的几何表现，事后整圈参考不是在线每帧的实际控制目标。</p>
<p>建议优先回看原始KCF在20:28:20和融合后视觉在20:26:05附近的峰值图像及状态，并按同帧配对表检查是否因观测、增益或状态变化导致偏离。若要比较策略，需尽量保持工件位置与运行条件一致重复采集，同时报告TCP与视觉峰值。</p>
<div class="actions"><a href="TCP_circle_error_components.csv">TCP逐点误差</a><a href="raw_KCF_circle_error_components.csv">原始KCF误差</a><a href="final_vision_circle_error_components.csv">融合后误差</a><a href="over_3mm_events.csv">峰值事件</a><a href="fusion_effects.csv">同帧融合影响</a><a href="axis_residual_summary.csv">各轴统计</a><a href="four_run_comparison.csv">四组对照</a><a href="trajectory_icp_effects.csv">轨迹ICP批次</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">数值复核</a><a href="browser_verification.json">浏览器检查</a></div>
<p>报告包含三维/XYZ交互图、静态预览、本地Plotly依赖和复现脚本，原始数据只读。复现依次运行本目录analyze.py、build_report.py、verify.py。浏览器采用无鼠标渲染与DOM事件检查；手动拖拽、缩放和平移未验收。输入哈希、独立标量几何、分量恒等式、事件计数和图表坐标分别校验。</p>
<footer>沿用此前读取的<a href="prior_notion_workflow_snapshot.json">Notion数据分析与协作工作流快照</a>（2026-10-19/18），不声称已核对在线最新版。未修改业务源码或此前分析、不删除文件、不使用鼠标；综合示教参考按用户确认保持一致。</footer></main>'''
write('report.html',page('自动卡尔曼在线纠偏｜20261020 202202｜移动后综合示教误差分析',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2))
print(json.dumps(dict(report=str(OUT/'report.html'),relative_vision_rmse_improvement_percent=relative_improvement),ensure_ascii=False,indent=2))
