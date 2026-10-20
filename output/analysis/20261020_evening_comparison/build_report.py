"""Build source-linked HTML report and a static preview from analyze.py outputs."""
from pathlib import Path
import hashlib, html, json
import numpy as np
import pandas as pd
from PIL import Image,ImageDraw,ImageFont
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
results=json.loads((OUT/'comparison.json').read_text());extra=[]
for r in results:
 p=OUT/r['session'][:19];q=pd.read_csv(p/'tcp_coordinates.csv');d=pd.read_csv(p/'diagnostics_enriched.csv',low_memory=False);dist=pd.read_csv(p/'TCP_to_raw_relocated_teach_distances.csv')
 times=q.timestamp_ms.to_numpy();dt=np.r_[np.diff(times),np.median(np.diff(times))]
 # Display-weight sensitivity: prolonged dropped frame gaps must not dominate; uncapped time weighting disclosed.
 tw=float(np.average(dist.distance_mm,weights=dt));c=np.array(r['raw_circle']['center']);n=np.array(r['raw_circle']['normal'])
 xyz=q[[a+'_material_frame_mm' for a in 'xyz']].to_numpy();delta=xyz-c;height=delta@n;radius=np.sqrt((delta*delta).sum(1)-height**2)
 angle=np.mod(np.rad2deg(np.arctan2(delta[:,1],delta[:,0])),360);bins=np.floor(angle/2).astype(int)
 anglemean=float(pd.DataFrame({'bin':bins,'d':dist.distance_mm}).groupby('bin').d.mean().mean())
 speed=np.abs(np.diff(np.rad2deg(np.unwrap(np.deg2rad(q.s2_deg))))/np.maximum(np.diff(times)/1000,1e-9))
 out=dict(session=r['session'],tcp_time_weighted_mean_mm=tw,tcp_angle_balanced_mean_mm=anglemean,tcp_radius_median_mm=float(np.median(radius)),tcp_normal_height_median_mm=float(np.median(height)),teach_minus_raw_circle_radius_mm=r['teach_circle']['radius_mm']-r['raw_circle']['radius_mm'],s2_speed_median_deg_s=float(np.median(speed)),first_60s_mean_mm=float(dist.distance_mm[times-times[0]<60000].mean()),after_60s_mean_mm=float(dist.distance_mm[times-times[0]>=60000].mean()),negative_state_age_percent=float(d.state_age_ms.lt(0).mean()*100))
 extra.append(out)
 d.assign(jump=np.maximum(d.d_final_x.abs(),d.d_final_y.abs())).nlargest(15,'jump').to_csv(p/'largest_pixel_steps.csv',index=False,encoding='utf-8-sig')
 # Signed geometric residuals to separate radial shape discrepancy from vertical misalignment.
 pd.DataFrame({'time':q.time,'radial_offset_from_teach_circle_mm':radius-r['teach_circle']['radius_mm'],'height_from_raw_kcf_plane_mm':height}).to_csv(p/'tcp_radial_height_diagnostics.csv',index=False,encoding='utf-8-sig')
(OUT/'sensitivity.json').write_text(json.dumps(extra,ensure_ascii=False,indent=2))
labels=['20:22 自动卡尔曼','20:41 仅 KCF','21:42 最新组']
def table(headers,rows):return '<table><thead><tr>'+''.join('<th>'+x+'</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+str(x)+'</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table>'
rows=[]
for label,r in zip(labels,results):
 m=r['metrics']['TCP_to_raw_relocated_teach'];rows.append([label,r['tcp_count'],r['raw_kcf_count'],f"{r['online_fps']:.2f}",f"{m['mean_mm']:.3f}",f"{m['p95_mm']:.3f}",f"{m['max_mm']:.3f}",r['summary']['session_state']])
comparison=table(['实验','有效 TCP','融合前 KCF','视觉帧/s','TCP→移动示教均值 mm','P95 mm','最大 mm','终态'],rows)
rows=[]
for label,r,x in zip(labels,results,extra):rows.append([label,f"{r['raw_circle']['radius_mm']:.3f}",f"{x['teach_minus_raw_circle_radius_mm']:.3f}",f"{x['tcp_radius_median_mm']:.3f}",f"{x['tcp_time_weighted_mean_mm']:.3f}",f"{x['tcp_angle_balanced_mean_mm']:.3f}",f"{x['s2_speed_median_deg_s']:.3f}"])
shape=table(['实验','KCF 描述圆半径 mm','示教半径－KCF半径 mm','TCP 中位半径 mm','时间加权距离 mm','2°扇区均衡距离 mm','S2中位速度 °/s'],rows)
rows=[]
for label,r in zip(labels,results):rows.append([label,f"{r['icp_applied']}/{r['icp_attempts']}",f"{r['icp_release_to_raw_max_mm']:.3f}",r['fusion_frames'],f"{r['fusion_displacement_mm']['median_mm']:.3f}",f"{r['fusion_displacement_mm']['max_mm']:.3f}",f"{r['final_x_jumps']}/{r['final_y_jumps']}",f"{r['jump_reinitialized']}/{r['jump_events']}"])
processing=table(['实验','轨迹ICP应用/尝试','已释放目标→批内原始折线最大 mm','融合帧数','融合位移中位 mm','融合位移最大 mm','|Δu|/|Δv|>5px次数','其中模板重建帧/事件'],rows)
latest=results[-1];s=extra[-1];m=latest['metrics']['TCP_to_raw_relocated_teach']
sections=[]
for label,r in zip(labels,results):
 dirname=r['session'][:19];seg=table(['JBI反馈行','TCP点数','开始','结束','平均 mm','最大 mm'],[[z['job_line'],z['n'],z['start'][11:],z['end'][11:],f"{z['metric']['mean_mm']:.3f}",f"{z['metric']['max_mm']:.3f}"] for z in r['segments']])
 sections.append(f'<section><h2>{label}</h2><p>{html.escape(r["session"])}<br>有效在线 TCP：{r["tcp_start"]}—{r["tcp_end"]}；S1/S2统一展示姿态={r["common_pose_deg"]}°。</p><p><a href="{dirname}/index.html">打开完整交互三维图</a> · <a href="{dirname}/metrics.csv">距离指标</a> · <a href="{dirname}/largest_pixel_steps.csv">像素跳变明细</a> · <a href="{dirname}/icp_release_effects.csv">轨迹 ICP 释放影响</a></p>{seg}<iframe loading="lazy" src="{dirname}/index.html" title="{label}三维轨迹"></iframe></section>')
report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>10月20日晚在线纠偏：三组对照分析</title>
<style>body{{font:16px/1.8 system-ui,"Microsoft YaHei",sans-serif;color:#203345;max-width:1280px;margin:30px auto;padding:0 22px}}h1{{font-size:27px}}h2{{font-size:21px;margin-top:30px}}table{{border-collapse:collapse;width:100%;margin:18px 0;font-size:14px}}td,th{{padding:9px;border-bottom:1px solid #d4dfe8;text-align:right}}td:first-child,th:first-child{{text-align:left}}a{{color:#0072b2}}iframe{{width:100%;height:850px;border:1px solid #d4dfe8}}.note{{background:#f4f8fb;padding:14px 18px}}img{{max-width:100%}}</style>
<h1>10月20日晚在线纠偏：三组对照分析</h1>
<p>核心判断：最新组基本完成一圈在线跟踪，TCP 与检测轨迹总体接近；相对刚体移动后的原始示教仍有约 2 mm 的系统性几何差异。当前三组不能用于证明局部定位 ICP 的增益，也不足以证明绝对焊缝精度。</p>
<p><a href="20261020_214257_216/index.html">直接打开最新组三维图</a> · <a href="comparison.csv">对照数据</a> · <a href="input_hashes.json">来源 SHA-256</a> · <a href="analyze.py">分析脚本</a> · <a href="build_report.py">报告脚本</a></p>
<h2>1. 对照结果与口径</h2>{comparison}
<p>主指标：有效在线 TCP 点到“融合前 KCF 定位后原始示教折线”的全部原始线段求最短三维距离，单位 mm。采用 online_running、corflag=1、detflag=1、s_start=1、s_hold=0、alarm=0 的 TCP，视觉采用 online_running 且 result_valid=1；剔除的仅为非统计阶段或无效状态，无异常点裁剪。蓝色 TCP、橙色融合前 KCF 都保留原始走势。</p>
<p>三组 TCP→原始融合前 KCF 折线平均距离分别为 {results[0]['metrics']['TCP_to_raw_KCF']['mean_mm']:.3f}、{results[1]['metrics']['TCP_to_raw_KCF']['mean_mm']:.3f}、{results[2]['metrics']['TCP_to_raw_KCF']['mean_mm']:.3f} mm。它是整圈几何贴近程度，既不按时间配对，也不代表因果跟随误差或延迟指标。</p>
<p>20:22 到 20:41 的主指标均值差 {results[1]['metrics']['TCP_to_raw_relocated_teach']['mean_mm']-results[0]['metrics']['TCP_to_raw_relocated_teach']['mean_mm']:.3f} mm；单次实验、视觉帧率及输入条件不同，不能把这点差别归因于卡尔曼。最新组更快，约 {s['s2_speed_median_deg_s']:.3f}°/s，前两组约 {extra[0]['s2_speed_median_deg_s']:.3f}/{extra[1]['s2_speed_median_deg_s']:.3f}°/s；不属于严格受控性能对比。</p>
<h2>2. 约 2 mm 距离主要表现为轮廓半径差</h2>{shape}
<p>原始示教描述圆半径统一为 {latest['teach_circle']['radius_mm']:.3f} mm；最新 KCF 为 {latest['raw_circle']['radius_mm']:.3f} mm，差 {s['teach_minus_raw_circle_radius_mm']:.3f} mm。移动示教只对齐圆心和法线，保留原半径、尺度=1；因此即使 TCP 贴近检测轨迹，仍会到绿色参考保持约 2 mm 的径向距离。三组均有这一模式，优先核对示教轮廓与当前识别特征是否对应同一条物理边缘，以及工具/传感器几何和标定。数据不能单独判定是哪一项造成。</p>
<p>描述圆只用于生成刚体定位参考和诊断半径，未替换实际示教折线、未平滑 TCP/KCF。时间加权和角度均衡均值见表，用于观察采样密度敏感性，不替换主指标。圆形轮廓绕法线的相位不可唯一识别，本分析不额外拟合周向转角。</p>
<h2>3. 两条 ICP 链路必须区分</h2>
<p class="note">目录中的“启动在线 ICP 但不成功 / 未启动在线 ICP”保留为现场说明。现有代码另有独立的工件局部定位 ICP（online-local-registration），它更新 RobotSim 工件位姿；online_icp_batches.txt 记录的则是在线轨迹 ICP，后者负责移动示教并释放当前批次的目标折线。这两种 ICP 开关与成功状态不能互相替代。</p>
{processing}
<p>三组日志均 enable_online_icp=true，表中的 applied 是轨迹 ICP 接受并释放目标；已释放目标对批内原始折线存在可量测的离开，说明并非只有状态字变化。该距离也包含重采样/模型约束影响，不是 ICP 准确度。无法由这些文件确认工件局部定位成功次数、位姿写回量或失败原因，当前会话未归档对应结构化记录。</p>
<p>当前代码依据：<code>docs/online_local_registration.md</code>，<code>Control/include/OnlineCorrectionTypes.h</code>，<code>Control/src/OnlineCorrectionExceptionProcessor.cpp</code>，<code>UI/src/OnlineCorrectionRunRecorder.cpp</code>。属于当前 workspace 对日志语义的解释；现场 exe 版本未冻结。</p>
<h2>4. 融合作用有限，像素突跳仍然存在</h2>
<p>20:41 全部 fusion_applied=0，融合前与最终视觉点完全一致；20:22、21:42 在线阶段分别有 {results[0]['fusion_frames']}、{latest['fusion_frames']} 个融合帧。最新组融合位移中位 {latest['fusion_displacement_mm']['median_mm']:.3f} mm，P95 {latest['fusion_displacement_mm']['p95_mm']:.3f} mm，最大 {latest['fusion_displacement_mm']['max_mm']:.3f} mm；以最终视觉重新定位参考后，TCP 平均距离为 {latest['metrics']['TCP_to_final_relocated_teach']['mean_mm']:.3f} mm，与主值 {m['mean_mm']:.3f} mm 很接近。</p>
<p>最新组共有 {latest['jump_events']} 对相邻在线帧满足 |Δu|&gt;5 px 或 |Δv|&gt;5 px，约 {100*latest['jump_events']/(latest['raw_kcf_count']-1):.2f}%；只有 {latest['jump_reinitialized']} 次与模板重建发生在同帧，不能把所有跳变归咎于模板更新。帧间隔中位 {latest['dt_ms']['median']:.0f} ms，P95 {latest['dt_ms']['p95']:.0f} ms。该阈值只是排查入口，包含真实运动、采样间隔和观测变化，并非真值误差阈值。</p>
<p>最新组最大融合前 KCF→移动示教距离 {latest['metrics']['raw_KCF_to_relocated_teach']['max_mm']:.3f} mm，时间 {latest['metrics']['raw_KCF_to_relocated_teach']['max_time']}；TCP 最大主距离 {m['max_mm']:.3f} mm，时间 {m['max_time']}。两者不同位置/时刻，不能直接相减或认定是同一事件。</p>
<h2>5. 最新组的结束状态与图像证据边界</h2>
<p>TCP 覆盖约 {latest['tcp_coverage_deg']:.2f}°，KCF 约 {latest['raw_kcf_coverage_deg']:.2f}°。21:52:16.801 生成 online_end_flag=1 的结束指令，21:52:16.932 记录完成回执；随后等待机器人程序停止，记录结束时 s_start 仍为 1、alarm=0、反馈程序行 21。21:52:28.153 被复位。应表述为“在线圈段结束并收到结束指令回执，但会话未进入正常完成终态”；不能直接写成轨迹失败或机器人已完全停止，也不能从该片段确定用户复位原因。</p>
<p>每组保存 30 对图像，最新组仅覆盖 {latest['image_first_time']}—{latest['image_last_time']}，主要是初始化与在线早期。21:44:30 的最大视觉偏离没有对应存档图像，不能据此确认是否识别错边。最新 camera_timestamp_ms − robot_state_received_ms 有 {s['negative_state_age_percent']:.1f}% 为负，说明采集帧可搭配随后收到的状态；不能把这列直接当作纯通信延迟，需结合冻结上下文和采集/处理时间核查同步策略。</p>
<h2>6. 坐标、来源与复现检查</h2>
<p>原示教沿用 {latest['work_path']}（1368点），由每组初始轨迹逐点确认其形状仅经历平移。机器人基座 TCP 经 RBCALIB 的 SRANG 变换及每行实际外轴逆变换回材料/变位机末节系；最终视觉的 TCP相对坐标恢复 [0,0,231] mm 后与此一致。融合前 KCF 根据 before_fusion 像素及同帧冻结姿态重投影，而不是把融合结果称为原始 KCF。</p>
<p>本地标定重算最终视觉固定基坐标最大差均小于 0.000002 mm，仅验证数学链一致性，不证明物理标定精度。展示再统一到各组首个有效 TCP 外轴姿态；Z 倍率仅改变显示纵横比，悬停读数保持真实毫米。绿色参考是事后整圈 KCF 定位，不能等同于在线控制器实际逐时目标或独立焊缝真值。</p>
<p>已核对汇总计数、记录唯一性、图像解码与全部源文件 SHA-256；未修改生产代码或原始记录。缺少现场冻结的标定/可执行程序/JBI 版本、独立焊缝测量、局部 ICP 状态历史；不做算法显著性或精度达标判定。源文件哈希见 input_hashes.json；几何函数、界面模板与 Plotly 已保存在 assets/，其哈希见 dependency_hashes.json。外部包 numpy、pandas、Pillow。</p>
<h2>7. 建议优先核查</h2>
<ol><li>先确认物理识别边缘与示教参考的对应关系，解释三组一致的约 2 mm 半径差。</li><li>补存工件局部 ICP 的开关、候选、拒绝原因、写回矩阵和回读结果；与轨迹 ICP 分开命名，才能验证开关效果。</li><li>保持速度、工件位置、JBI 和相机设置一致，重复自动融合/仅KCF对照；按同一几何口径比较。</li><li>图像记录改为覆盖整圈或异常触发留图，并核对时间戳与机器人姿态匹配；本次仅分析，未修改配置或代码。</li></ol>
{''.join(sections)}</html>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
print(json.dumps(extra,ensure_ascii=False,indent=2))
