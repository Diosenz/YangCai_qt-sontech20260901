"""Build reviewable offline deliverables from already computed metrics."""
from pathlib import Path
import hashlib, html, importlib.util, json, sys
sys.dont_write_bytecode=True
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
ASSETS=ROOT/'output/analysis/20261020_evening_comparison/assets'
spec=importlib.util.spec_from_file_location('geometry',ASSETS/'geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
def points(name):return np.loadtxt(OUT/name,delimiter=',',skiprows=1,encoding='utf-8-sig')
work=points('original_teach.csv');moved=points('estimated_moved_teach.csv')
tq=points('tcp_material_frame.csv')[:,1:];kq=points('raw_kcf_material_frame.csv')[:,1:]
def circlepoints(c):
    n=np.array(c['normal']);u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u);v=np.cross(n,u)
    a=np.linspace(0,2*np.pi,1441)
    return c['center']+c['radius_mm']*(np.cos(a)[:,None]*u+np.sin(a)[:,None]*v)
R=np.array(r['alignment']['rotation']);t=np.array(r['alignment']['translation'])
fit=circlepoints(r['original_circle'])
traces=[]
def trace(q,name,color,visible=True,mode='lines',dash='solid'):
    traces.append(dict(type='scatter3d',mode=mode,name=name,x=q[:,0].tolist(),y=q[:,1].tolist(),z=q[:,2].tolist(),visible=visible,line=dict(color=color,width=3,dash=dash),marker=dict(color=color,size=2),hovertemplate=name+'<br>X=%{x:.5f} mm<br>Y=%{y:.5f} mm<br>Z=%{z:.5f} mm<extra></extra>'))
trace(tq,'TCP 实测轨迹','#0072b2');trace(kq,'KCF 融合前检测点','#d55e00')
trace(fit,'原始示教拟合圆（辅助）','#777777',False,dash='dash')
trace(fit@R.T+t,'移动后示教拟合圆（辅助）','#009e73',False)
trace(work,'原始示教实际点与连线','#777777',mode='lines+markers',dash='dash')
trace(moved,'移动后示教实际点与连线（估计参考）','#009e73',mode='lines+markers')
allq=np.vstack([np.c_[a['x'],a['y'],a['z']] for a in traces]);lo=allq.min(0);hi=allq.max(0);span=hi-lo
template=(ASSETS/'viewer_template.html').read_text(encoding='utf-8')
template=template.replace('/* PLOTLY_LIBRARY */',(ASSETS/'plotly-2.35.2.min.js').read_text(encoding='utf-8')).replace('/* TRACE_DATA */',json.dumps(traces,ensure_ascii=False)).replace('/* AXIS_RANGES */',json.dumps(np.c_[lo-span*.06,hi+span*.06].tolist())).replace('/* AXIS_SPANS */',json.dumps(span.tolist()))
template=template.replace('20261019_110509_938 · 固定变位机基坐标系 · 统一 S1/S2 = COMMON_POSE','20261021_152328_050 · 变位机末节材料坐标系 · 每点已消除自身 S1/S2 旋转')
template=template.replace('<option value="actual">','<option value="actual" selected>').replace('<option value="fit" selected>','<option value="fit">')
a=template.index('<footer>');z=template.index('</footer>',a)
footer=f'<footer><a href="report.html">返回分析报告</a> · TCP {len(tq):,} 点；原始 KCF {len(kq):,} 点；两组示教各 {len(work):,} 点。未平滑或剔除观测点。绿色由 KCF 整圈圆心/法线定位得到，保持原始半径与形状，是估计参考，不能直接当独立真值。仅拟合圆视图不会改变误差统计。Z 倍率仅改变显示，不改变坐标或计算。本轮未进行真实浏览器交互验收。</footer>'
template=template[:a]+footer+template[z+len('</footer>'):]
(OUT/'index.html').write_text(template,encoding='utf-8')
(OUT/'viewer_data.json').write_text(json.dumps(dict(traces=traces,ranges=np.c_[lo-span*.06,hi+span*.06].tolist(),spans=span.tolist()),ensure_ascii=False),encoding='utf-8')
def table(rows,columns):
    return '<table><thead><tr>'+''.join('<th>'+html.escape(label)+'</th>' for key,label in columns)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(f'{row[key]:.4f}' if isinstance(row[key],float) else str(row[key]))+'</td>' for key,label in columns)+'</tr>' for row in rows)+'</tbody></table>'
metric_labels={'TCP_to_estimated_moved_teach':'TCP → 移动后示教（估计参考）','raw_KCF_to_estimated_moved_teach':'原始 KCF → 移动后示教（估计参考）','final_vision_to_estimated_moved_teach':'融合后视觉 → 同一估计参考（补充）','TCP_to_raw_KCF_polyline':'TCP → 原始 KCF 折线（补充）'}
rows=[dict(name=label,**r['metrics'][key]) for key,label in metric_labels.items()]
metric_table=table(rows,[('name','比较对象'),('n','点数'),('mean_mm','平均距离/mm'),('max_mm','最大距离/mm'),('rmse_mm','RMSE/mm'),('p95_mm','P95/mm'),('max_time','最大值时间')])
wc=r['original_circle'];kc=r['raw_kcf_circle'];tc=r['tcp_circle'];delta=np.array(kc['center'])-wc['center']
def vec(v):return '('+', '.join(f'{x:.6f}' for x in v)+')'
cv=r['cross_validation_pooled'];sd=r['cross_validation_reference_displacement'];loc=r['local_registration']
alternatives=table(r['reference_alternatives'],[('reference','移动方式'),('trajectory','评价轨迹'),('mean_mm','均值/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm')])
segments=table(r['tcp_phase_metrics'],[('phase','阶段'),('start','开始'),('end','结束'),('n','点数'),('mean_mm','均值/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm')])
shape=table([dict(name=name,radius_mm=c['radius_mm'],cx=c['center'][0],cy=c['center'][1],cz=c['center'][2]) for name,c in [('原始示教',wc),('原始KCF',kc),('TCP',tc)]],[('name','数据'),('radius_mm','描述性拟合半径/mm'),('cx','圆心X/mm'),('cy','圆心Y/mm'),('cz','圆心Z/mm')])
report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>15:23 在线纠偏复测分析</title><style>body{{max-width:1200px;margin:35px auto;padding:0 25px;font:16px/1.8 system-ui,"Microsoft YaHei";color:#203345}}h1{{font-size:28px}}h2{{font-size:21px;margin-top:32px}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{border-bottom:1px solid #dce5ec;padding:10px;text-align:left}}th{{background:#edf4fa}}.note{{background:#fff5df;padding:14px 18px;border-left:4px solid #d58b00}}.good{{background:#eaf5f0;padding:14px 18px}}code,pre{{font-size:14px;background:#f5f7f9}}pre{{padding:15px;overflow:auto}}a{{color:#0072b2}}img{{max-width:100%}}</style>
<h1>在线纠偏复测：四条轨迹与误差分析</h1><p>数据：2026-10-21 15:23:28–15:33:38；会话 <code>{r['session']}</code>。<a href="index.html">打开四轨迹交互三维图</a> · <a href="metrics.csv">统计表</a> · <a href="results.json">完整结果与限制</a></p>
<div class="good">本轮 TCP 与 KCF 的几何路径接近。以本报告的刚体移动示教为估计参考，TCP 平均距离 <b>1.7110 mm</b>、最大 <b>4.1262 mm</b>、RMSE <b>1.8506 mm</b>；原始 KCF 分别为 <b>1.7929、5.1076、1.9314 mm</b>。</div>
<p class="note"><b>当前无法建立独立焊缝真值。</b>用户确认没有移动后的示教文件或独立工件移动参数。以下绿色参考依据 KCF 估计位置，不能把其误差直接称为绝对定位/检测精度。原始示教与本轮 KCF 还有约 <b>{abs(r['radius_difference_kcf_minus_original_mm']):.4f} mm</b> 的拟合半径差，刚体移动无法消除；原因仍可能涉及工件形状差异、特征定义、检测偏差或标定。</p>
<h2>1. 以什么为依据移动示教</h2>
<table><tr><th>候选依据</th><th>调查证据</th><th>本次判断</th></tr><tr><td>独立测量的工件位姿/二次示教</td><td>用户确认没有相应文件和参数</td><td>这是后续建立独立真值的首选，当前缺失</td></tr><tr><td>RobotSim 的工件位姿</td><td>当前生产代码把 virtualToReal 左乘当前工件位姿，并回写1068；本轮 {loc['batches']} 批归档、{loc['verified_writebacks']} 次通过独立新帧验收的回写</td><td>属于局部表面ICP和视觉验收驱动的估计，且随运行变化；不能当真实工件的已知位姿</td></tr><tr><td>启动时段首点对齐</td><td>只把反扫末点对齐示教首点，缺少整圈约束；TCP到该参考均值 {r['metrics']['TCP_to_startup_teach']['mean_mm']:.3f} mm</td><td>用于现有启动流程，不能作为事后整圈焊缝真值</td></tr><tr><td>用TCP轨迹对齐示教</td><td>TCP正是被评价的执行结果</td><td>会吸收跟随偏差，不采用</td></tr><tr><td>用原始KCF整圈圆心/法线定位</td><td>与机器人执行轨迹分开，覆盖约 {r['raw_kcf_coverage_deg']:.2f}°；能约束中心和平面</td><td><b>作为当前可复算的估计参考采用</b>，保持原始半径、点形状与点序，补做参考敏感性和留出验证</td></tr></table>
<p>不是把示教变成 KCF 拟合圆。仅在<b>估计移动参数</b>时，对 2° 角度分箱的中位代表点拟合圆心/法线，降低驻留密度与局部跳点对定位的影响；所有原始观测仍用于最终误差统计，未平滑、未剔除异常点。示教的全部 {r['original_count']} 个实际点和原有线段均保留；不缩放、不改半径、不做非刚性变形。</p>
<p>原始示教圆心 {vec(wc['center'])} mm → 当前 KCF 圆心 {vec(kc['center'])} mm，<b>圆心位移 {vec(delta)} mm</b>；法线最小夹角 {r['alignment']['normal_alignment_deg']:.6f}°。近圆形数据不能可靠确定绕自身法线的相位角，本次选择最小旋转，不额外拟合绕轴转角。原始轨迹的小幅非圆特征仍保留，但其物理相位尚未经独立验证。</p>
<p>完整变换采用列向量 <code>p_new = R p_old + t</code>；矩阵中的平移 t={vec(t)} mm 与“圆心位移”不同，因为旋转也会改变圆心位置。<a href="teach_to_estimated_current_transform.csv">下载4×4刚体矩阵</a>；<a href="estimated_moved_teach.csv">下载移动后示教全部点</a>。</p>
<pre>{html.escape(np.array2string(np.block([[R,t[:,None]],[np.zeros((1,3)),np.ones((1,1))]]),precision=9,suppress_small=True))}</pre>
<h2>2. 坐标系、筛选与误差定义</h2>
<p>四组轨迹统一到变位机末节的材料坐标系，单位 mm。机器人基坐标先按 RBCALIB 转到变位机基坐标，再逐点消除 Ry(S1)Rz(S2) 的外轴旋转；视觉原有父级TCP原点相对末节有 Z=231 mm 偏置，+231 mm 后与原始 W 文件一致。这里的末节材料系与工件父级1063 TCP系只差固定原点，不能混用。依据当前 <code>PositionerKinematics.cpp</code>、<code>WeldProjectionContextBuilder.cpp</code> 和 <code>OnlineCorrectionTrajectoryPlanner.cpp</code> 复核。</p>
<p>TCP：在线运行且 CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0，共 {r['tcp_count']} 点。KCF：online_running 且 result_valid=1，共 {r['raw_kcf_count']} 点，使用 <code>before_fusion_x/y_px</code> 和同帧冻结机器人姿态重新投影。融合后的 final_x/y 不冒充原始 KCF。原始记录共 {r['summary']['robot_state_count']} 条机器人状态、{r['summary']['vision_diagnostic_count']} 条视觉记录，启动/等待/停止阶段不纳入主要统计。没有额外去重或异常值裁剪。</p>
<pre>e_i = min over every recorded reference segment [a_j,b_j] ||p_i - closest_point(p_i,[a_j,b_j])||_2
平均误差 = sum(e_i)/N
最大误差 = max(e_i)
RMSE = sqrt(sum(e_i^2)/N)</pre>
<p>使用<b>点到实际示教折线所有有限线段的最短三维距离</b>，不按行号或时间硬配对，不改为最近采样点或到拟合圆的距离，不自动补闭合线段。均值按记录样本加权；这衡量路径几何贴合，不衡量沿缝时序滞后或姿态角误差。TCP与KCF不同采样数量分别统计。</p>
{metric_table}
<p>TCP→KCF 折线只是补充的几何一致性指标，不是 TCP 的绝对误差；相邻 KCF 异常点的连线可能影响最近距离。融合后视觉采用相同参考，便于观察融合影响，不参与参考定位。</p>
<h2>3. 主要差异与稳健性检查</h2>{shape}
<p>TCP与KCF拟合半径仅相差 {tc['radius_mm']-kc['radius_mm']:.4f} mm，二者圆心相差 {np.linalg.norm(np.array(tc['center'])-kc['center']):.4f} mm。但KCF半径比原始示教小 {abs(r['radius_difference_kcf_minus_original_mm']):.4f} mm。它与约1.7–1.8 mm的参考误差量级相近，提示应优先查明“同一真实焊缝、同一特征、同一形状”的前提；不能把全部差值直接解释为控制没跟上，也不能通过缩放示教消掉差值后宣称高精度。</p>
<h3>只平移与平移+法线对齐</h3>{alternatives}<p>两种移动方式得到的均值差约0.02 mm，说明本轮主要统计不依赖这0.168°小倾角。</p>
<h3>KCF分扇区留出验证</h3><p>沿整圈每20°交替划为两组。用一组估计参考，评价另一组，交换后合并。留出KCF均值 <b>{cv['mean_mm']:.4f} mm</b>、最大 <b>{cv['max_mm']:.4f} mm</b>、RMSE <b>{cv['rmse_mm']:.4f} mm</b>；两个留出参考相对全量参考的逐点位移最大约 <b>{sd['max_mm']:.4f} mm</b>。这支持对齐参数对局部采样有一定稳定性，<b>不能消除共享标定和检测系统偏差，也不能把估计参考升级为真值</b>。见<a href="raw_kcf_cross_validation.csv">逐点留出结果</a>。</p>
<h3>局部配准前后仅作描述</h3>{segments}<p>首次通过验收前只有36个TCP点，且前后处在不同焊缝位置；该分段不构成局部ICP改善量的受控实验，不能将均值下降全部归因于ICP。各机器人程序行的统计见<a href="tcp_job_line_metrics.csv">分段表</a>。</p>
<img src="preview.png" alt="四轨迹投影、误差随时间和误差累计分布"><p><a href="index.html">交互图</a>提供3D/XY/XZ/YZ视角、两组示教独立的点/线/拟合圆切换、Z显示倍率。倍率只改变显示比例，误差始终基于真实坐标。</p>
<h2>4. 本轮结论与下一步</h2><p>本轮有效在线区间覆盖整圈，TCP与检测轨迹总体一致，约1.7–1.8 mm的主要参考差异仍包含明显形状/半径成分。与旧轮相同估计方法相比，TCP均值由3.75降至1.71 mm、最大由15.17降至4.13 mm；工件摆放、轨迹和运行条件没有严格控制，因此仅说明各自归档的描述性结果变化，不能作为单一工具修复或局部ICP的因果增益。</p>
<p><b>当前建议：</b>本轮保留上述刚体移动参考用于诊断，明确标注“估计参考”。若要报告严格的TCP/KCF绝对焊缝精度，应在当前摆放下重新示教焊缝，或测量原摆放与当前摆放中至少三个不共线固定特征点，以独立刚体变换移动原始示教。先核对原示教和本次工件是否确为相同半径、相同焊缝定义；即便取得独立位姿，工件形状不同也不能仅靠刚体移动建立准确真值。</p>
<h2>5. 验证、输入与复现</h2><p>复算全部final视觉点与归档工站坐标的最大差 {r['verification']['final_projection_max_mm']:.3g} mm；恢复末节坐标后与归档PositionerTcp+231的最大差 {r['verification']['positioner_replay_max_mm']:.3g} mm；{r['verification']['raw_kcf_archive_replay_count']} 帧原始KCF与局部归档独立焊点的最大差 {r['verification']['raw_kcf_archive_replay_max_mm']:.3g} mm。刚体矩阵正交性、行列式和所有示教线段长度保持均已断言。输入SHA-256见<a href="input_hashes.json">哈希清单</a>。</p>
<p>原始输入：<code>output/online_correction/{r['session']}/</code>；局部配准归档：<code>output/online_local_registration/20261021_072328_003_session_1/</code>；示教：<code>data/WorkPiece/20260404CHUFENGKOU1_W.txt</code>；当前标定：<code>config/calibration/RBCALIB.DAT</code>、<code>SENSORPARA.txt</code>。当前标定可重现归档，但没有完整历史标定快照，绝对标定精度不由上述数值闭环保证。</p>
<p>参考了前一对话已获取的<a href="https://app.notion.com/p/3feaa04fa8a9816c9f51c32fad802a46">Notion数据分析展示偏好</a>（2026-10-19快照）和<a href="https://app.notion.com/p/3fdaa04fa8a9819195d2dc0eb65ce630">AI协作工作流</a>（2026-10-18快照），见<a href="prior_notion_workflow_snapshot.json">历史快照</a>。本次在线刷新不可用，未修改Notion。本轮只新增派生分析产物，未修改业务源码或原始数据，未删除文件、使用鼠标或发送设备指令。静态图已检查；真实浏览器旋转/缩放操作尚未验收。</p>
<p>复现：使用已提供numpy/pandas/Pillow的项目分析Python，依次运行同目录 <code>analyze.py</code>、<code>build_report.py</code>、<code>verify.py</code>。图表所用Plotly为既有本地资产，HTML已内嵌该依赖，可离线打开。</p></html>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
hashes=json.loads((OUT/'input_hashes.json').read_text(encoding='utf-8'))
for p in [ASSETS/'viewer_template.html',ASSETS/'plotly-2.35.2.min.js',ROOT/'docs/online_local_registration.md',ROOT/'Vision/src/geometry/PositionerKinematics.cpp',ROOT/'Vision/src/geometry/WeldProjectionContextBuilder.cpp',ROOT/'Control/src/OnlineCorrectionTrajectoryPlanner.cpp']:
    hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'input_hashes.json').write_text(json.dumps(hashes,ensure_ascii=False,indent=2),encoding='utf-8')
print('Report and six-trace viewer generated.')
