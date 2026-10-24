"""Create the automatic-fusion report using the previously checked visual template."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261020_204102_kcf_only_composite_teach'
source=(TEMPLATE/'build_report.py').read_text(encoding='utf-8').split('comparison_rows=[]',1)[0]
source=source.replace('10月20日20:41','10月20日20:22').replace('仅KCF','自动卡尔曼').replace('最终视觉（与KCF相同）','卡尔曼融合后视觉')
source=source.replace("ev[ev.trajectory.ne('final_vision')].sort_values",'ev.sort_values')
source+='''
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
body=f\'''<main><span class="badge">2026-10-20 · 20:22会话 · 自动卡尔曼 · 固定综合示教参考</span>
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
<footer>沿用此前读取的<a href="prior_notion_workflow_snapshot.json">Notion数据分析与协作工作流快照</a>（2026-10-19/18），不声称已核对在线最新版。未修改业务源码或此前分析、不删除文件、不使用鼠标；综合示教参考按用户确认保持一致。</footer></main>\'''
write('report.html',page('自动卡尔曼在线纠偏｜20261020 202202｜移动后综合示教误差分析',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2))
print(json.dumps(dict(report=str(OUT/'report.html'),relative_vision_rmse_improvement_percent=relative_improvement),ensure_ascii=False,indent=2))
'''
(OUT/'build_report.py').write_text(source,encoding='utf-8')
print('Prepared automatic-Kalman report template.')
