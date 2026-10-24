"""Publish the fixed-reference report; retain every superseded report."""
from pathlib import Path
import hashlib,html,json,shutil
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent
OLD=OUT.parent/'20261021_152328_050_retest'
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
for directory in [OUT,OLD]:
    original=directory/'report.html';backup=directory/'report_before_final_synthesized_reference.html'
    if original.exists() and not backup.exists():shutil.copy2(original,backup)
circle=r['current_circle'];c=np.array(circle['center']);n=np.array(circle['normal']);n/=np.linalg.norm(n)
u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u);v=np.cross(n,u)
def stamp(t):return pd.to_datetime(t,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3]
def stat(e):return dict(n=len(e),mean_mm=float(e.mean()),max_mm=float(e.max()),rmse_mm=float(np.sqrt(np.mean(e*e))),p95_mm=float(np.quantile(e,.95)))
data={};components=[];sectors=[];events=[];input_hashes={}
for label in ['TCP','raw_KCF','final_vision']:
    p=OUT/(label+'_to_synthesized_circle.csv');input_hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    df=pd.read_csv(p);q=df[['x_mm','y_mm','z_mm']].to_numpy();d=q-c
    h=d@n;rho=np.linalg.norm(np.cross(d,n),axis=1);signed_radial=rho-circle['radius_mm'];e=np.hypot(signed_radial,h)
    assert np.max(abs(e-df.distance_mm))<1e-10
    a=r['metrics'][label]
    assert abs(e.mean()-a['mean_mm'])<1e-12 and abs(np.sqrt(np.mean(e**2))-a['rmse_mm'])<1e-12
    angle=np.mod(np.rad2deg(np.arctan2(d@v,d@u)),360.)
    components.append(dict(trajectory=label,radial_mean_mm=float(signed_radial.mean()),radial_rmse_mm=float(np.sqrt(np.mean(signed_radial**2))),plane_height_mean_mm=float(h.mean()),plane_height_rmse_mm=float(np.sqrt(np.mean(h*h)))))
    for sector in range(12):
        mask=(angle>=30*sector)&(angle<30*(sector+1))
        if mask.any():sectors.append(dict(trajectory=label,angular_sector=f'{30*sector}–{30*(sector+1)}°',**stat(e[mask])))
    over=e>3;starts=np.where(over&~np.r_[False,over[:-1]])[0];ends=np.where(over&~np.r_[over[1:],False])[0]
    for start,end in zip(starts,ends):
        # A missing-record gap over 200 ms splits a run into separate events.
        cuts=np.where(np.diff(df.timestamp_ms.iloc[start:end+1])>200)[0]+start
        for first,last in zip(np.r_[start,cuts+1],np.r_[cuts,end]):
            maximum=first+int(np.argmax(e[first:last+1]));events.append(dict(trajectory=label,start=stamp(df.timestamp_ms.iloc[first]),end=stamp(df.timestamp_ms.iloc[last]),span_ms=int(df.timestamp_ms.iloc[last]-df.timestamp_ms.iloc[first]),point_count=int(last-first+1),max_mm=float(e[maximum]),max_time=stamp(df.timestamp_ms.iloc[maximum]),signed_radial_at_max_mm=float(signed_radial[maximum]),plane_height_at_max_mm=float(h[maximum]),angle_at_max_deg=float(angle[maximum])))
    df['signed_radial_mm']=signed_radial;df['signed_plane_height_mm']=h;df['material_angle_deg']=angle
    df.to_csv(OUT/(label+'_circle_error_components.csv'),index=False,encoding='utf-8-sig');data[label]=df
pd.DataFrame(components).to_csv(OUT/'error_component_summary.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(events).to_csv(OUT/'over_3mm_events.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(sectors).to_csv(OUT/'angular_sector_metrics.csv',index=False,encoding='utf-8-sig')
sens=pd.read_csv(OUT/'leave_one_teaching_out_sensitivity.csv')
labelmap={'TCP':'TCP实际轨迹','raw_KCF':'KCF融合前检测轨迹','final_vision':'融合后视觉（补充）'}
def table(rows,cols):
    return '<div class="table-wrap"><table><thead><tr>'+''.join('<th>'+title+'</th>' for key,title in cols)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(f'{row[key]:.4f}' if isinstance(row[key],float) else str(row[key]))+'</td>' for key,title in cols)+'</tr>' for row in rows)+'</tbody></table></div>'
main=[]
for label,m in r['metrics'].items():main.append(dict(name=labelmap[label],**m,within2=f"{100*m['within_2mm_fraction']:.2f}%",within3=f"{100*m['within_3mm_fraction']:.2f}%"))
main_table=table(main,[('name','轨迹'),('n','有效点数'),('mean_mm','平均误差/mm'),('max_mm','最大误差/mm'),('rmse_mm','RMSE/mm'),('p95_mm','P95/mm'),('within2','≤2 mm'),('within3','≤3 mm')])
source_rows=[dict(name=s['name']+'_W.txt',n=s['W_points'],tn=s['T_points'],radius=s['circle']['radius_mm'],weight='25%') for s in r['sources']]
sources_table=table(source_rows,[('name','历史原始示教'),('n','W点数'),('tn','配套T点数'),('radius','单独拟合半径/mm'),('weight','综合权重')])
comp_table=table([dict(name=labelmap[a['trajectory']],**a) for a in components],[('name','轨迹'),('radial_mean_mm','径向有符号均值/mm'),('radial_rmse_mm','径向RMSE/mm'),('plane_height_mean_mm','离面有符号均值/mm'),('plane_height_rmse_mm','离面RMSE/mm')])
k_events=[e for e in events if e['trajectory']=='raw_KCF'];peak=max(k_events,key=lambda e:e['max_mm'])
event_table=table(sorted(k_events,key=lambda e:e['max_mm'],reverse=True)[:10],[('start','开始'),('end','结束'),('point_count','点数'),('span_ms','首末跨度/ms'),('max_mm','最大/mm'),('signed_radial_at_max_mm','峰值径向/mm'),('plane_height_at_max_mm','峰值离面/mm')])
sens_rows=[]
for label in ['TCP','raw_KCF']:
    a=sens[sens.trajectory.eq(label)];sens_rows.append(dict(name=labelmap[label],mean_range=f"{a.mean_mm.min():.3f}–{a.mean_mm.max():.3f}",rmse_range=f"{a.rmse_mm.min():.3f}–{a.rmse_mm.max():.3f}",max_range=f"{a.max_mm.min():.3f}–{a.max_mm.max():.3f}"))
sens_table=table(sens_rows,[('name','轨迹'),('mean_range','平均误差范围/mm'),('rmse_range','RMSE范围/mm'),('max_range','最大误差范围/mm')])
prior=pd.read_csv(OUT/'reference_comparison.csv')
comparison=table([dict(name=labelmap[a['trajectory']],reference='上一版单组示教圆' if a['reference']=='previous_single_teach_circle' else '本版四组综合示教圆',**{k:a[k] for k in ['mean_mm','max_mm','rmse_mm']}) for a in prior.to_dict('records') if a['trajectory']!='final_vision'],[('name','轨迹'),('reference','参考定义'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm')])
source_links=' · '.join(f'<a href="sources/{Path(s["W_file"]).stem}/{Path(s["W_file"]).name}">{s["name"]}原始W</a>' for s in r['sources'])
tcp=r['metrics']['TCP'];kcf=r['metrics']['raw_KCF'];rad=r['new_reference_radius_mm']
base='20261021CHUFENGKOU20_SYNTH_ANALYSIS'
report=f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>在线纠偏复测报告｜综合新示教圆参考</title><style>
:root{{--ink:#203345;--muted:#526576;--blue:#0072b2;--line:#dce5ec}}*{{box-sizing:border-box}}body{{margin:0;background:#f5f7fa;color:var(--ink);font:16px/1.8 system-ui,"Microsoft YaHei",sans-serif}}main{{max-width:1260px;margin:28px auto;background:white;padding:36px 42px;border-radius:12px}}h1{{font-size:29px;line-height:1.4;margin:8px 0 12px}}h2{{font-size:22px;margin-top:36px;border-bottom:1px solid var(--line);padding-bottom:8px}}h3{{font-size:18px}}p{{margin:12px 0}}a{{color:var(--blue)}}.meta,.muted{{color:var(--muted)}}.badge{{font-size:13px;background:#e9f3fb;color:#00649b;padding:5px 9px;border-radius:5px}}.lead{{background:#eaf5f0;padding:18px 22px;border-left:4px solid #009e73}}.context{{background:#f2f6fa;padding:15px 20px}}.cards{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:20px 0}}.card{{border:1px solid var(--line);padding:16px;border-radius:7px}}.card b{{display:block;font-size:27px;line-height:1.4;color:var(--blue)}}.card span{{font-size:13px;color:var(--muted)}}.table-wrap{{overflow-x:auto}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{padding:10px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}}th{{background:#edf4fa}}figure{{margin:22px 0}}img{{max-width:100%;height:auto}}figcaption{{font-size:13px;color:var(--muted)}}code,pre{{font-size:14px;background:#f4f6f8}}pre{{padding:15px;white-space:pre-wrap;overflow:auto}}.actions{{display:flex;gap:12px;flex-wrap:wrap}}.actions a{{text-decoration:none;padding:8px 14px;border:1px solid #a8c4d8;border-radius:6px}}footer{{margin-top:35px;padding-top:15px;border-top:1px solid var(--line);font-size:13px;color:var(--muted)}}@media(max-width:750px){{main{{margin:0;padding:22px 16px;border-radius:0}}h1{{font-size:25px}}.cards{{grid-template-columns:repeat(2,1fr)}}}}@media print{{body{{background:white}}main{{max-width:none;margin:0;padding:0}}.actions{{display:none}}h2{{break-after:avoid}}tr,figure,.lead{{break-inside:avoid}}}}
</style></head><body><main><span class="badge">固定新综合参考 · 2026-10-21 正式分析版</span>
<h1>在线纠偏复测：综合新示教圆参考下的轨迹误差分析</h1>
<p class="meta">工件：出风口小（正20mm）｜会话：20261021_152328_050｜采集时间：2026-10-21 15:23:28–15:33:38｜单位：mm</p>
<div class="actions"><a href="index.html">打开四轨迹交互三维图</a><a href="metrics.csv">下载主统计表</a><a href="{base}_W.txt">新原始示教_W</a><a href="{base}_T.txt">新原始示教_T</a></div>
<div class="cards"><div class="card"><span>TCP平均误差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>KCF平均误差</span><b>{kcf['mean_mm']:.3f} mm</b><span>RMSE {kcf['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP在3 mm内</span><b>{100*tcp['within_3mm_fraction']:.2f}%</b><span>最大 {tcp['max_mm']:.3f} mm</span></div><div class="card"><span>KCF在3 mm内</span><b>{100*kcf['within_3mm_fraction']:.2f}%</b><span>超过3 mm：{kcf['above_3mm_count']}点</span></div></div>
<div class="lead"><b>结论：</b>以四组历史原始示教综合的新圆、经KCF定位移动后作为参考，本轮TCP和KCF的平均误差均小于0.8 mm，RMSE均约0.9 mm，P95均小于1.6 mm。结合用户认可的约2～3 mm误差量级，<b>本轮整体几何贴合表现可接受</b>。TCP全部有效点在3 mm内；KCF少量局部偏离仍保留，最大6.471 mm，不能把平均较小理解为所有检测点均无异常。</div>
<p class="context"><b>本版唯一主参考：</b>四组历史示教分别拟合后等权综合，半径固定为 <b>{rad:.6f} mm</b>；再把圆心、法线对齐本轮融合前KCF估计的位置。综合尺寸未使用本轮TCP或KCF测量；KCF只参与位置/倾角对齐，TCP只参与评价。所有统计均为点到<b>连续三维圆周</b>的解析最短距离。</p>
<h2>1. 数据范围与四条轨迹</h2><p>沿用同一轮15:23复测，未切换至后来采集的其它会话。TCP保留在线运行且CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0的{tcp['n']:,}个有效点；KCF保留online_running且result_valid=1的{kcf['n']:,}个融合前点。两者均使用已验证的坐标转换结果，未对测量轨迹平滑、拟合或剔除峰值。融合后的最终视觉点仅列作补充对照。</p>
<p>四条主曲线依次为：灰色虚线——综合新原始示教拟合圆；蓝色——TCP实际轨迹；橙色——KCF融合前检测点轨迹；绿色——移动后的综合新示教拟合圆。统一使用变位机末节材料坐标系；逐点消除各自S1/S2旋转，并将视觉父级TCP的231 mm原点偏置还原到与示教相同的坐标原点。</p>
<figure><img src="preview.png" alt="综合示教圆、TCP、KCF与移动后综合圆的空间投影及误差"><figcaption>空间投影采用等比例；交互图提供3D/XY/XZ/YZ视角和Z显示倍率。显示倍率不改变坐标、圆参数或误差计算。</figcaption></figure>
<h2>2. 新原始示教如何建立</h2>{sources_table}
<p>只纳入有正面工件映射依据的四种W几何，按XYZ内容去重，不按备份数量增加权重。每组W先估计圆心，按2°周向分箱取XYZ中位代表点，再拟合最小二乘平面和平面内的圆，并迭代降低局部径向偏离点对拟合的影响。配套T用于来源核对与模板保留，不作为另一组完整圆观测重复计权。</p>
<p>各组历史圆先按圆心/法线刚体归一，不缩放。新圆半径取四组半径的算术平均，每组25%；新原始圆心、法线沿用当前项目原始示教的参考摆放。再将当前W/T的XYZ投影到该新圆，形成另存的分析参考文件。<b>具体顺序是“分别拟合历史圆 → 综合圆参数 → 生成新W/T”</b>，并非把四组不同摆放位置的XYZ直接混在一起拟合。</p>
<p>{source_links}</p><p>来源、配对与去重依据：<a href="all_chufeng_W_T_inventory.csv">完整检索清单</a> · <a href="selection_decisions.csv">纳入/排除清单</a> · <a href="historical_mappings.json">工件映射证据</a>。反面工件、早期不完整圆弧及对应关系不明确的其它文件未混入主综合。四组半径范围140.648～143.717 mm，代表不同历史记录的尺寸差异；不宣称它们来自同一件实物或完全相同的标定状态。</p>
<h2>3. 圆的移动方式与误差定义</h2><p>综合圆半径保持{rad:.6f} mm，使用保持尺度不变的刚体变换，将其圆心和法线对齐本轮原始KCF拟合圆。移动后圆心为({c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f}) mm；单位法线为({n[0]:.9f}, {n[1]:.9f}, {n[2]:.9f})。没有改用KCF自身半径，也没有用TCP寻优参考位置。圆周几何不依赖绕圆法线的相位角。</p>
<pre>c：移动后的圆心；n：圆平面单位法线；R：综合半径
h = (p-c)·n
ρ = ||(p-c)-h n||
e = sqrt((ρ-R)² + h²)
平均误差 = mean(e)；最大误差 = max(e)；RMSE = sqrt(mean(e²))</pre>
<p>该距离同时包含径向和离面偏差，评价对象是空间圆周；绘图的离散采样线段不参与统计。不按时间或行号强行配对TCP与KCF，均值按各自记录样本加权，不评价沿缝时序滞后和工具姿态角。圆参数及变换：<a href="synthesized_original_circle.json">移动前</a> · <a href="moved_synthesized_circle.json">移动后</a> · <a href="teach_to_current_transform.csv">刚体矩阵</a>。</p>
<h2>4. 主误差结果与2～3 mm范围</h2>{main_table}
<p>TCP有{100*tcp['within_2mm_fraction']:.2f}%、KCF有{100*kcf['within_2mm_fraction']:.2f}%的有效点在2 mm内；两条轨迹的P95分别为{tcp['p95_mm']:.3f}、{kcf['p95_mm']:.3f} mm。这里按用户给出的约2～3 mm可接受量级进行描述，不将3 mm擅自升级为严格的逐点工艺限值。</p>
<p>逐点结果：<a href="TCP_circle_error_components.csv">TCP距离及分量</a> · <a href="raw_KCF_circle_error_components.csv">KCF距离及分量</a> · <a href="final_vision_circle_error_components.csv">融合后视觉补充结果</a>。</p>
<h2>5. 局部偏离与误差分量</h2>{comp_table}
<p>径向和离面均值为有符号量，可能相互抵消；综合评价仍使用前表的非负三维距离及RMSE。当前参考圆心/法线由KCF对齐，整体位置和倾角偏差可能被吸收，不能把这些分量解释成独立标定误差。</p>
<p>KCF共{kcf['above_3mm_count']}点超过3 mm，占{100*(1-kcf['within_3mm_fraction']):.3f}%，分成{len(k_events)}个记录连续事件。事件按相邻原始记录均超3 mm归组；相邻时间间隔超过200 ms再拆分，表中“首末跨度”只是已有记录时间跨度，不推断采样空隙内的真实状态。下表列误差最大的最多10个事件，完整记录见<a href="over_3mm_events.csv">事件表</a>。</p>{event_table}
<p>最大KCF偏离发生在 <b>{peak['max_time']}</b>，三维距离 <b>{peak['max_mm']:.3f} mm</b>，当时径向分量 {peak['signed_radial_at_max_mm']:.3f} mm、离面分量 {peak['plane_height_at_max_mm']:.3f} mm。这说明存在局部几何偏离，但仅凭这些轨迹不能确定是图像检测跳点、实际焊缝局部形状、机器人状态时效还是其它原因。当前报告不将它直接归为控制故障。TCP最大偏离{tcp['max_mm']:.3f} mm，发生在{tcp['max_time']}。</p>
<p>另按圆平面角度每30°分段统计，见<a href="angular_sector_metrics.csv">空间分段表</a>。角度零点为圆平面内的投影X方向，用于定位空间区段，不代表机器人程序起始相位。</p>
<h2>6. 参考敏感性与上一版对照</h2><p>每次剔除四组历史示教中的一组，用其余三组等权重建圆，得到如下参考敏感性范围。主报告始终使用预先确定的四组等权结果，不从这些候选中选择误差最小者。</p>{sens_table}<p><a href="leave_one_teaching_out_sensitivity.csv">查看完整敏感性表</a>。这些变化说明误差数值确实受历史参考尺寸选择影响，不能脱离参考定义单独比较数值大小。</p>{comparison}
<p><b>本版较上一版数值减小，是参考尺寸改变后的重新评价。</b>TCP、KCF测量没有变化，不能据此声称控制算法或本轮执行过程又提升了一次。仿真模型由测绘获得、真实工件制造误差较大的背景下，历史半径差保留为参考来源差异，不再作为需要优先排查的纠偏异常。</p>
<h2>7. 结论与使用边界</h2><p>以当前固定的新综合示教圆为参考，本轮整体几何贴合达到用户认可的误差量级：TCP全部有效点在3 mm内，KCF有99.67%有效点在3 mm内；主要剩余现象是KCF少量局部峰值。当前结果可用于比较整圈几何贴合、识别局部偏离并支撑本次复测总结。</p>
<p>新圆的尺寸独立于本轮测量，但位置/倾角仍由KCF估计，因此它是<b>历史综合比较参考</b>，不能据此宣称当前实物已取得独立绝对焊缝真值。新W/T仅用于分析，姿态、外轴和时间列沿用旧模板，未重新验证为可执行机器人示教程序；生产配置未切换。</p>
<h2>8. 验证与可追溯附件</h2><p>已核对历史W/T拷贝哈希、四组几何去重、权重和为1、刚体变换、生成点在综合圆上，以及逐点解析距离的独立交叉计算。主统计表与逐点CSV回读一致；TCP/KCF与此前同轮派生坐标一致。静态图已检查，交互图脚本及控制逻辑检查通过，真实浏览器交互未验收。</p>
<p><a href="verification.json">数据与综合验证</a> · <a href="circle_viewer_verification.json">图表逻辑验证</a> · <a href="input_hashes.json">综合输入哈希</a> · <a href="final_report_input_hashes.json">本版报告输入哈希</a> · <a href="results.json">完整圆参数及元信息</a>。</p>
<footer>本版基于同一轮15:23数据和同一新综合参考整理，未改业务源码、原始示教、外部备份或运行配置，未删除文件，未使用鼠标或发送设备指令。旧版报告已保留快照。复现本版报告：运行本目录build_final_report.py。</footer></main></body></html>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
# Keep the screenshot's old entry point useful, with all links resolving to the
# canonical report directory. Both previous report bodies remain backed up.
entry=report.replace('<head>','<head><base href="../20261021_positive20_teach_synthesis/">',1)
(OLD/'report.html').write_text(entry,encoding='utf-8')
for p in [OUT/'results.json',OUT/'metrics.csv',OUT/'leave_one_teaching_out_sensitivity.csv',OUT/'reference_comparison.csv',OUT/'viewer_data.json']:
    input_hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'final_report_input_hashes.json').write_text(json.dumps(input_hashes,ensure_ascii=False,indent=2),encoding='utf-8')
summary=dict(session=r['session'],reference_radius_mm=rad,reference_circle='moved_synthesized_circle.json',KCF_over_3mm_event_count=len(k_events),KCF_over_3mm_point_count=kcf['above_3mm_count'],KCF_peak_event=peak,error_components=components,primary_entry=str(OUT/'report.html'),updated_previous_entry=str(OLD/'report.html'),raw_measurements_changed=False)
(OUT/'final_report_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
