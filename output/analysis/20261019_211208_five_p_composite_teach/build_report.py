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
labels={'TCP':'TCP 实际轨迹','recorded_vision':'记录视觉输出（融合状态未知）'}
colors={'TCP':'#0072b2','recorded_vision':'#d55e00'}
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
for key in ['TCP','recorded_vision']:
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
for(const [id,fit,actual] of [['original-mode',2,4],['relocated-mode',3,5]])document.getElementById(id).addEventListener('change',e=>{const k=e.target.value;Plotly.restyle('plot',{visible:[k==='fit'||k==='both',k!=='fit']},[fit,actual]);Plotly.restyle('plot',{mode:k==='points'?'markers':'lines+markers'},[actual]);});
for(const b of document.querySelectorAll('[data-view]'))b.addEventListener('click',()=>{const eyes={iso:{x:1.5,y:1.5,z:.9},xy:{x:0,y:0,z:2.2},xz:{x:0,y:-2.2,z:0},yz:{x:2.2,y:0,z:0}};Plotly.relayout('plot',{'scene.camera':{eye:eyes[b.dataset.view],up:b.dataset.view==='xy'?{x:0,y:1,z:0}:{x:0,y:0,z:1}}});});'''.replace('TRACES',json.dumps(traces,ensure_ascii=False)).replace('LAYOUT',json.dumps(layout)).replace('BASE',json.dumps(base.tolist()))
write('index.html',page('10月19日21:12｜综合示教参考四轨迹三维图','<main style="max-width:none;margin:0;padding:16px"><h2 style="margin:0">10月19日21:12｜CFKONTEST3五个P点 · 四轨迹空间对照</h2><p>变位机末节随动系，单位mm。灰色/绿色均为此前综合示教参考，半径142.035340 mm；生成W点不是新实测点。蓝色TCP、橙色记录视觉保留全部有效点。移动后参考为事后估计，不能当独立真值。</p><p>'+controls+'</p><div id="plot" style="height:730px"></div><p class="meta">显示切换不改变误差定义：统计始终使用连续综合圆周。Z倍率不改变任何坐标或误差。可旋转、缩放、平移；<a href="report.html" target="_top">返回报告</a>。</p></main>','<script src="plotly-2.35.2.min.js"></script><script>'+viewer_js+'</script>'))
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
write('axis_fluctuations.html',page('10月19日21:12｜XYZ方向误差波动','<main style="margin:0;padding:14px;max-width:none"><h2 style="margin:0">相对移动后综合示教圆的 XYZ 偏差</h2><p>测量点减去圆周最近点；无平滑、无剔除。各轴分量用于看波动，三维距离=√(ΔX²+ΔY²+ΔZ²)。拖选放大，双击复位。</p><div id="axes"></div></main>','<script src="plotly-2.35.2.min.js"></script><script>window.axisViewerReady=Plotly.newPlot("axes",'+json.dumps(axis_traces,ensure_ascii=False)+','+json.dumps(axis_layout)+',{responsive:true,displaylogo:false,scrollZoom:true});</script>'))
# Time windows reveal startup concentration without suppressing peaks.
windows=[]
for key,d in data.items():
    relative=(d.timestamp_ms-origin)/1000
    for name,mask in [('前60秒',relative<60),('60秒以后',relative>=60)]:
        if mask.any():windows.append(dict(trajectory=labels[key],window=name,**g.metric(d.distance_mm[mask]),above_3mm=int((d.distance_mm[mask]>3).sum())))
pd.DataFrame(windows).to_csv(OUT/'time_window_metrics.csv',index=False,encoding='utf-8-sig')
# Static overview with true coordinates; XZ remains equal scale.
im=Image.new('RGB',(1700,1150),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',23);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18)
draw.text((45,15),'10月19日21:12 在线纠偏｜CFKONTEST3五个P点｜固定综合示教参考',fill='#203345',font=font)
draw.text((45,55),'蓝：TCP  橙：记录视觉  绿：移动后综合圆  灰：移动前综合圆；无测量点删减',fill='#526576',font=small)
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
spatial=[(circle_points(r['original_circle']),'#777777'),(data['TCP'][['x_mm','y_mm','z_mm']].to_numpy(),colors['TCP']),(data['recorded_vision'][['x_mm','y_mm','z_mm']].to_numpy(),colors['recorded_vision']),(circle_points(r['current_circle']),'#009e73')]
chart((110,170,790,540),[(q[:,0],q[:,1],c) for q,c in spatial],'XY / mm（等比例）',True)
chart((980,170,1640,540),[(q[:,0],q[:,2],c) for q,c in spatial],'XZ / mm（等比例）',True)
chart((110,700,790,1060),[((data[k].timestamp_ms-origin)/1000,data[k].distance_mm,colors[k]) for k in ['TCP','recorded_vision']],'全部有效在线点：时间/s → 三维误差/mm')
chart((980,700,1640,1060),[(np.sort(data[k].distance_mm),np.arange(1,len(data[k])+1)/len(data[k]),colors[k]) for k in ['TCP','recorded_vision']],'距离/mm → 累计比例')
im.save(OUT/'preview.png')
main_rows=[dict(name=labels[k],**a,within2=f"{100*a['within_2mm_fraction']:.2f}%",within3=f"{100*a['within_3mm_fraction']:.2f}%") for k,a in r['metrics'].items()]
metric_table=table(main_rows,[('name','轨迹'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('p95_mm','P95/mm'),('within2','≤2 mm'),('within3','≤3 mm'),('above_3mm_count','>3 mm点数')])
circles=[dict(name='固定综合原始示教（移动前）',**r['original_circle']),dict(name='移动后综合示教参考',**r['current_circle'])]+[dict(name=labels[k]+'拟合圆',**c) for k,c in r['fitted_circles'].items()]
circle_table=table([dict(name=c['name'],x=c['center'][0],y=c['center'][1],z=c['center'][2],radius_mm=c['radius_mm'],normal=', '.join(f'{a:.9f}' for a in c['normal'])) for c in circles],[('name','圆'),('x','圆心X/mm'),('y','圆心Y/mm'),('z','圆心Z/mm'),('radius_mm','半径/mm'),('normal','单位法线')])
ev=pd.read_csv(OUT/'over_3mm_events.csv');event_table=table([dict(name=labels[a['trajectory']],**a) for a in ev.sort_values('max_mm',ascending=False).head(15).to_dict('records')],[('name','轨迹'),('start','开始'),('end','结束'),('count','点数'),('max_mm','最大/mm'),('radial_at_peak_mm','峰值径向/mm'),('plane_at_peak_mm','峰值离面/mm')])
component_table=table([dict(name=labels[a['trajectory']],**a) for a in r['components']],[('name','轨迹'),('radial_mean_mm','径向有符号均值/mm'),('radial_rmse_mm','径向RMSE/mm'),('plane_mean_mm','离面有符号均值/mm'),('plane_rmse_mm','离面RMSE/mm'),('delta_x_rmse_mm','ΔX RMSE/mm'),('delta_y_rmse_mm','ΔY RMSE/mm'),('delta_z_rmse_mm','ΔZ RMSE/mm')])
window_table=table(windows,[('trajectory','轨迹'),('window','时段'),('n','点数'),('mean_mm','平均/mm'),('rmse_mm','RMSE/mm'),('max_mm','最大/mm'),('above_3mm','>3 mm点数')])
sens=pd.read_csv(OUT/'leave_one_teaching_out_sensitivity.csv');sens_rows=[]
for key in ['TCP','recorded_vision']:
    a=sens[sens.trajectory.eq(key)];sens_rows.append(dict(name=labels[key],mean=f'{a.mean_mm.min():.3f}–{a.mean_mm.max():.3f}',rmse=f'{a.rmse_mm.min():.3f}–{a.rmse_mm.max():.3f}',maximum=f'{a.max_mm.min():.3f}–{a.max_mm.max():.3f}'))
sens_table=table(sens_rows,[('name','轨迹'),('mean','留一组示教：平均范围/mm'),('rmse','RMSE范围/mm'),('maximum','最大范围/mm')])
source_table=table([dict(name=a['name'],n=a['W_points'],radius=a['circle']['radius_mm'],weight='25%') for a in json.loads((PREV/'results.json').read_text(encoding='utf-8'))['sources']],[('name','历史示教'),('n','W点数'),('radius','拟合半径/mm'),('weight','综合权重')])



tcp=r['metrics']['TCP'];vision=r['metrics']['recorded_vision'];peak=ev.sort_values('max_mm',ascending=False).iloc[0]
comparison_rows=[];report_hashes={}
for directory,label in [('20261019_211208_five_p_composite_teach','10/19 21:12 五个P点'),('20261020_202202_auto_kalman_composite_teach','10/20 20:22 自动卡尔曼'),('20261020_204102_kcf_only_composite_teach','10/20 20:41 仅KCF'),('20261021_positive20_teach_synthesis','10/21 15:23 自动卡尔曼'),('20261021_163204_sim01_kcf09_composite_teach','10/21 16:32 固定0.1/0.9')]:
    path=OUT.parent/directory/'metrics.csv';previous=pd.read_csv(path);report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for key,description in [('TCP','TCP'),('recorded_vision' if directory==OUT.name else 'final_vision','记录视觉（融合状态未知）' if directory==OUT.name else '最终视觉')]:
        a=previous[previous.trajectory.eq(key)].iloc[0]
        comparison_rows.append(dict(session=label,name=description,n=int(a.n),mean_mm=float(a.mean_mm),max_mm=float(a.max_mm),rmse_mm=float(a.rmse_mm),within3=f"{a.within_3mm_fraction*100:.2f}%"))
pd.DataFrame(comparison_rows).to_csv(OUT/'five_run_output_comparison.csv',index=False,encoding='utf-8-sig')
comparison_table=table(comparison_rows,[('session','会话'),('name','轨迹类型'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('within3','≤3 mm')])
body=f'''<main><span class="badge">2026-10-19 · 21:12会话 · CFKONTEST3五个P点 · 综合示教参考</span>
<h1>五个P点在线纠偏测试：<br>移动后综合示教参考下的轨迹误差分析</h1>
<p class="meta">会话 {html.escape(r['session'])}｜会话21:12:08–21:18:47｜有效在线约21:13:00–21:16:47（北京时间）｜长度单位mm</p>
<div class="actions"><a href="index.html">独立打开三维图</a><a href="axis_fluctuations.html">XYZ波动图</a><a href="metrics.csv">主统计表</a><a href="results.json">完整分析数据</a></div>
<div class="cards"><div class="card"><span>TCP平均误差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>记录视觉平均误差</span><b>{vision['mean_mm']:.3f} mm</b><span>RMSE {vision['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大误差</span><b>{tcp['max_mm']:.3f} mm</b><span>{tcp['above_3mm_count']}点超过3 mm</span></div><div class="card"><span>记录视觉最大误差</span><b>{vision['max_mm']:.3f} mm</b><span>{vision['above_3mm_count']}点超过3 mm</span></div></div>
<div class="lead"><b>结论：</b>固定此前综合示教，按本组记录视觉估计位姿并移动后比较，TCP和视觉的平均距离约1.36～1.38 mm，RMSE约1.54～1.56 mm。TCP有{tcp['above_3mm_count']}点超过3 mm，视觉有{vision['above_3mm_count']}点，局部偏离保留。相较随后几组会话，本组平均误差和RMSE较高，但不能将差异直接归因于五个P点。<b>本组缺少融合前观测与增益记录，且会话以停止等待超时结束，几何表现和流程完成状态需分开判断。</b></div>
<p class="note"><b>固定参考：</b>仍是此前四组历史示教等权综合的原始示教圆，半径 <b>142.035339635 mm</b>。本组只根据已有的记录视觉拟合圆心与法线，刚体移动该综合示教，不缩放、不用TCP寻优、不重选半径。由于没有融合前KCF诊断，本组参考的位姿来源与后续“原始KCF定位”的报告有所不同；几何距离定义相同，但证据条件不完全相同。移动后参考不是独立真实焊缝测量。</p>
<h2>1. 原始数据、五个P点与结束状态</h2><p>本会话只有session_summary、actual_robot_path、initial_correction和realtime_correction四类文本。机器人记录{r['source_counts']['robot']:,}条，视觉采样{r['source_counts']['vision_samples']:,}条。TCP按online_running、CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0筛选，保留{tcp['n']:,}点；视觉按vision_sample、online_running、vision_valid=1筛选，保留{vision['n']:,}点。未平滑、未删除有效峰值。</p>
<p>initial_correction中有<b>{r['source_counts']['initial_teach_points']}条teach_point初始命令</b>，与session_summary的initial_command_count=5一致，原始记录另存为<a href="five_p_initial_commands.csv">五个初始点命令表</a>。这确认了五点发送记录；没有完整的历史JBI/程序版本快照，不能只靠目录名证明其它运行条件与后续各组相同。当前综合参考来自既有历史综合，不将这五个控制点当成五组独立焊缝测量重复计权。</p>
<p>有效TCP：{r['valid_window']['tcp_start']}–{r['valid_window']['tcp_end']}；有效视觉：{r['valid_window']['vision_start']}–{r['valid_window']['vision_end']}。排除waiting_robot_program_stop阶段的{r['source_counts']['waiting_stop_tcp']:,}条机器人记录。最终session_state=error，原始消息如下：</p><pre>{html.escape(r['summary']['message'])}</pre>
<p>该记录说明程序在等待停止确认时超时，不能把本会话写成流程正常完成；也不能仅凭超时就判定此前在线运动的几何数据全部无效。本报告只评价已记录的有效在线段，不覆盖停止等待阶段或未记录路径。</p>
<h2>2. 视觉记录的含义与坐标</h2><p>realtime_correction中的vision_x/y/z保存视觉结果点；其消息属于KCF/Gabor检测链路，但没有before_fusion像素、融合标记、增益、原始图像或同帧机器人姿态诊断，<b>不能拆出原始KCF与融合后视觉，也不能据此认定“仅KCF”或“自动卡尔曼”模式</b>。本报告统一命名“记录视觉输出（融合状态未知）”。session_summary有enable_online_icp=true，但没有轨迹ICP批次明细或局部表面配准/写回归档，无法核实应用次数与实际位姿修正效果。</p>
<p>坐标统一到<b>变位机末节随动坐标系（材料系）</b>。TCP先由RBCALIB从机器人基系转固定站系，再逐点消除Ry(S1)Rz(S2)；记录视觉已经在变位机工具TCP系，Z加回231 mm恢复末节原点。initial_correction里的tcp_offset三轴记录与(0,0,231)一致。视觉坐标不再使用插值外轴重复旋转，不为材料系距离强行配对TCP时间。</p>
<p>视觉timestamp_ms是记录时间，缺少独立camera_timestamp_ms，时间图仅作为记录顺序/时段观察，不解释为已校准的采集时延。切换工具TCP系时只需所有圆心Z减231 mm，半径和误差不变。</p>
<h2>3. 四轨迹三维图与XYZ波动</h2><p>蓝：TCP；橙：记录视觉输出；灰色虚线：移动前综合示教；绿：移动后综合示教。支持三维、XY、XZ、YZ视角、两组示教独立切换和Z显示倍率，默认1×等比例。生成W点是综合参考采样，不是新增实测点；倍率不改变坐标或误差。</p>
<iframe src="index.html" title="四轨迹交互三维图" style="height:1040px" loading="eager"></iframe>
<h3>测量点相对参考圆最近点的ΔX、ΔY、ΔZ</h3><p>下图是各测量点减去圆周最近点，不是TCP与视觉按同一时刻的差；无平滑、无剔除。</p><iframe src="axis_fluctuations.html" title="XYZ波动图" style="height:990px" loading="lazy"></iframe>
<figure><img src="preview.png" alt="真实坐标等比例投影、全部有效点误差与累计分布"><figcaption>测量轨迹按记录顺序连线，不补缺口、不强制闭合。完整圆仅为参考几何。</figcaption></figure>
<h2>4. 综合示教来源、移动及误差结果</h2>{source_table}<p>沿用同一四组综合圆，不重新拟合本组半径。原始综合圆心/法线不变，移动后与本组记录视觉拟合中心/法线一致；半径不变。绕法线相位不可辨识，采用最小旋转。下表TCP和视觉拟合圆仅用于描述，不替代原始轨迹。</p>{circle_table}
{metric_table}<pre>c = 移动后综合圆心；n = 单位法线；R = 固定综合半径142.035339635 mm
h = (p-c)·n；ρ = ||(p-c)-h n||
e = √((ρ-R)²+h²)
平均 = mean(e)；最大 = max(e)；RMSE = √mean(e²)</pre><p>与后续综合报告相同，使用到<b>连续三维综合圆周</b>的解析最短距离，不混用原单组示教折线、最近采样点或行号配对误差。统计按记录样本加权，不按弧长/时间加权，不评价姿态角或沿缝时滞。3 mm用于观察偏离，不擅自定义工艺合格线。</p>
<h2>5. 误差分量、时间分布与峰值</h2>{component_table}<p>有符号径向/离面均值可能抵消，整体评价以非负三维距离和RMSE为准。参考位姿由记录视觉估计，可吸收整体位置/倾角偏差，不能把分量当作独立标定误差。</p>{window_table}<p>前60秒从保留轨迹的最早时间起算，不是会话启动时刻；所有有效起始点均保留。TCP最大值发生在{tcp['max_time']}；记录视觉最大值发生在{vision['max_time']}。峰值事件按相邻记录都超3 mm归组，记录间隔超过200 ms拆分，下表最多列15段。</p>{event_table}
<p>最大记录视觉偏离为{peak['max_mm']:.3f} mm，发生在{peak['max_time']}，径向分量{peak['radial_at_peak_mm']:.3f} mm、离面分量{peak['plane_at_peak_mm']:.3f} mm。没有该时刻的图像与融合前诊断，不能据此断言是KCF跳点、融合、真实焊缝局部形状或机器人状态延迟造成。完整事件与作业行指标均保留在附件中。</p>
<h2>6. 覆盖缺口与参考敏感性</h2><p>有效视觉周向覆盖约<b>{r['coverage_deg']:.3f}°</b>，最大未采样扇区约<b>{360-r['coverage_deg']:.3f}°</b>，首末记录点距离{r['visual_endpoint_gap_mm']:.3f} mm。图中不补闭合线，不把未观测扇区当成测得的路径。参考圆是连续几何，因此仍可定义有效点到参考的距离，但无法证明缺口区域的真实误差或完整闭环表现。</p>
<p>交错20°扇区留出：一半估计中心/法线，另一半评价，记录视觉合并RMSE为{r['holdout']['pooled']['rmse_mm']:.3f} mm，半径始终固定。留出降低同点重复使用，仍不能消除共享视觉和标定偏差。以下历史示教留一组敏感性仅作边界展示，不据此挑选更好半径。</p>{sens_table}
<h2>7. 与后续四组会话的描述对照</h2>{comparison_table}<p>统一参考几何和误差公式，但本组位姿来自融合状态未知的记录视觉，后续各组来自融合前KCF。为避免把早期输出冒充原始KCF，表中视觉行对照后续的<b>最终视觉输出</b>；存在观测条件和参考定位来源差异，不是完全同条件实验。</p>
<p>本组TCP与记录视觉的平均误差/RMSE均高于后续已分析的四组。不同批次、有效运动时长、观测覆盖、程序状态及标定历史可能共同影响结果；不能据此认定五个P点方案必然较差，或推断某个ICP/融合开关带来了确定改善。若要研究五点方案，应在相同工件位置和运行条件下进行配对重复测试。</p>
<h2>8. 复核、附件与后续建议</h2><p>TCP坐标正逆变换最大回算差{r['verification']['tcp_frame_roundtrip_max_mm']:.2e} mm；与此前独立保存的本会话坐标结果对照，TCP和视觉最大差分别{r['verification']['prior_saved_tcp_max_difference_mm']:.2e}、{r['verification']['prior_saved_vision_max_difference_mm']:.2e} mm。初始修正后的1368点相对原工作轨迹保形平移残差{r['verification']['initial_corrected_path_shape_max_difference_mm']:.2e} mm。这里复核坐标解释的一致性，不把历史分析当成独立真值。</p>
<p>建议优先核对21:13:09附近的TCP偏离和21:16:29附近的视觉峰值，并单独排查21:18:47的停止确认超时。该会话没有原始图像/融合诊断，无法在现有记录上完成同帧像素回放或融合前后归因；如需继续比较五P点效果，应补充这些记录，并保留运行时JBI和标定版本。综合圆仍是事后估计参考，不是独立测得的移动后焊缝。</p>
<div class="actions"><a href="TCP_circle_error_components.csv">TCP逐点误差</a><a href="recorded_vision_circle_error_components.csv">记录视觉逐点误差</a><a href="over_3mm_events.csv">峰值事件</a><a href="tcp_job_line_metrics.csv">作业行指标</a><a href="axis_residual_summary.csv">各轴统计</a><a href="five_run_output_comparison.csv">五组输出对照</a><a href="five_p_initial_commands.csv">五点初始命令</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">数值复核</a><a href="browser_verification.json">浏览器检查</a></div>
<p>离线HTML包含三维和XYZ交互图、静态预览、Plotly本地依赖及生成脚本。复现依次运行本目录analyze.py、build_report.py、verify.py。浏览器采用无鼠标渲染及DOM事件检查，手动拖拽/缩放/平移未验收。所有新增产物在本目录，原始输入和先前分析只读。</p>
<footer>沿用已有<a href="prior_notion_workflow_snapshot.json">Notion数据分析与协作工作流快照</a>（2026-10-19/18），未重新声称读取在线最新版。不使用鼠标、不删除文件、不修改业务源码；参考按用户确认固定为此前综合原始示教。</footer></main>'''
write('report.html',page('CFKONTEST3五个P点｜20261019 211208｜移动后综合示教误差分析',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2))
print(json.dumps(dict(report=str(OUT/'report.html'),windows=windows),ensure_ascii=False,indent=2))
