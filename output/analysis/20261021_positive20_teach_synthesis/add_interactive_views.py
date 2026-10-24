"""Embed the four-trajectory 3D view and add per-axis geometric residual plots."""
from pathlib import Path
import hashlib,html,json
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
OLD=OUT.parent/'20261021_152328_050_retest'
datasets={}
for key in ['TCP','raw_KCF']:
    d=pd.read_csv(OUT/(key+'_to_synthesized_circle.csv'))
    delta=d[['x_mm','y_mm','z_mm']].to_numpy()-d[['foot_x_mm','foot_y_mm','foot_z_mm']].to_numpy()
    assert np.max(abs(np.linalg.norm(delta,axis=1)-d.distance_mm.to_numpy()))<1e-10
    datasets[key]=(d,delta)
origin=min(d.timestamp_ms.iloc[0] for d,delta in datasets.values())
traces=[];summary=[]
for axis_index,axis in enumerate('XYZ'):
    for key,label,color in [('TCP','TCP','#0072b2'),('raw_KCF','KCF 融合前','#d55e00')]:
        d,delta=datasets[key];values=delta[:,axis_index]
        stamp=pd.to_datetime(d.timestamp_ms,unit='ms',utc=True).dt.tz_convert('Asia/Shanghai').dt.strftime('%H:%M:%S.%f').str[:-3]
        traces.append(dict(type='scattergl',mode='lines',name=label,legendgroup=key,showlegend=axis_index==0,x=((d.timestamp_ms-origin)/1000).tolist(),y=values.tolist(),xaxis='x' if axis_index==0 else 'x'+str(axis_index+1),yaxis='y' if axis_index==0 else 'y'+str(axis_index+1),text=stamp.tolist(),line=dict(color=color,width=1.2),hovertemplate=label+'<br>时间 %{text}<br>Δ'+axis+'=%{y:.5f} mm<extra></extra>'))
        summary.append(dict(trajectory=key,axis=axis,mean_mm=float(values.mean()),rmse_mm=float(np.sqrt(np.mean(values**2))),maximum_absolute_mm=float(np.max(abs(values)))))
pd.DataFrame(summary).to_csv(OUT/'axis_residual_summary.csv',index=False,encoding='utf-8-sig')
layout=dict(height=840,margin=dict(l=70,r=25,t=50,b=55),font=dict(family='Microsoft YaHei, sans-serif',color='#203345'),legend=dict(orientation='h',x=0,y=1.06),hovermode='x',paper_bgcolor='#ffffff',plot_bgcolor='#fafcfe')
for i,axis in enumerate('XYZ'):
    suffix='' if i==0 else str(i+1);domain=[[.72,1],[.36,.64],[0,.28]][i]
    layout['yaxis'+suffix]=dict(domain=domain,title='Δ'+axis+' / mm',zeroline=True,zerolinecolor='#526576',gridcolor='#e0e6eb')
    layout['xaxis'+suffix]=dict(anchor='y'+suffix,title='在线时间 / s' if i==2 else '',showticklabels=i==2,matches='x3' if i!=2 else None,gridcolor='#e0e6eb')
library=(ROOT/'output/analysis/20261020_evening_comparison/assets/plotly-2.35.2.min.js').read_text(encoding='utf-8')
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>XYZ方向偏差与波动</title><style>body{margin:0;color:#203345;font:15px/1.6 system-ui,"Microsoft YaHei"}header{padding:12px 20px}h2{font-size:21px;margin:0 0 6px}p{margin:4px 0;color:#526576}#axes{width:100%;height:840px}</style><header><h2>相对移动后综合示教圆的 XYZ 偏差</h2><p>ΔX/ΔY/ΔZ = 测量点 − 参考圆最近点；不是TCP与KCF同一时刻的配对差。曲线保留全部有效点，无平滑、无剔除。</p><p>拖选放大、双击复位；点击图例切换TCP或KCF。三维误差为 sqrt(ΔX²+ΔY²+ΔZ²)，各轴阈值不能代替三维距离阈值。<a href="report.html" target="_top">返回报告</a></p></header><div id="axes"></div><script>'''+library+'</script><script>const traces='+json.dumps(traces,ensure_ascii=False)+';const layout='+json.dumps(layout,ensure_ascii=False)+';window.axisViewerReady=Plotly.newPlot("axes",traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});</script></html>'
(OUT/'axis_fluctuations.html').write_text(page,encoding='utf-8')
(OUT/'axis_viewer_data.json').write_text(json.dumps(dict(traces=traces,layout=layout),ensure_ascii=False),encoding='utf-8')
snippet='''<section id="interactive-views"><h2>三维轨迹与各轴波动：交互观察</h2><p>先用三维视角观察整圈，再切换XY、XZ、YZ逐方向查看。Z向显示放大只改变视觉比例，1×可恢复XYZ等比例；原始TCP、KCF及真实坐标读数不变。</p><p class="actions"><a href="index.html" target="_blank">独立打开三维图</a><a href="axis_fluctuations.html" target="_blank">独立打开XYZ波动图</a><a href="axis_residual_summary.csv">下载各轴统计</a></p><iframe src="index.html" title="四轨迹交互三维图" style="width:100%;height:980px;border:1px solid #dce5ec;border-radius:8px" loading="eager"></iframe><h3>相对参考圆的各轴偏差随时间变化</h3><p>下图分别展示每个测量点相对参考圆最近点的ΔX、ΔY、ΔZ。它将整圈运行造成的大尺度坐标变化与几何偏差区分开，便于识别局部波动；它不是TCP与KCF按时间配对后的差值。</p><iframe src="axis_fluctuations.html" title="TCP和KCF的XYZ偏差波动" style="width:100%;height:1030px;border:1px solid #dce5ec;border-radius:8px" loading="lazy"></iframe></section>'''
report=(OUT/'report.html').read_text(encoding='utf-8')
report=report.replace('复现本版报告：运行本目录build_final_report.py。','复现本版报告：依次运行本目录build_final_report.py和add_interactive_views.py。')
if '<section id="interactive-views">' in report:
    start=report.index('<section id="interactive-views">');end=report.index('</section>',start)+len('</section>');report=report[:start]+snippet+report[end:]
else:
    position=report.index('<figure>');report=report[:position]+snippet+report[position:]
(OUT/'report.html').write_text(report,encoding='utf-8')
(OLD/'report.html').write_text(report.replace('<head>','<head><base href="../20261021_positive20_teach_synthesis/">',1),encoding='utf-8')
(OUT/'axis_viewer_verification.json').write_text(json.dumps(dict(trace_count=len(traces),source_counts={key:len(d) for key,(d,delta) in datasets.items()},axis_residual_norm_matches_3d_error=True,coordinate_frame='positioner last-link/material frame',measurement_smoothing=False,mouse_used=False,actual_browser_interaction='not performed'),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(report_embeds=['index.html','axis_fluctuations.html'],summary=summary),ensure_ascii=False,indent=2))
