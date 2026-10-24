"""Adapt only the presentation template; conclusions come from this run's metrics."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261021_163204_sim01_kcf09_composite_teach'
source=(TEMPLATE/'build_report.py').read_text(encoding='utf-8').split("body=f'''",1)[0]
source=source.replace('16:32','10月20日20:41').replace('仿真0.1 / KCF0.9','仅KCF').replace('KCF 融合前','KCF 原始检测').replace('融合后视觉','最终视觉（与KCF相同）').replace('紫色融合后视觉','紫色最终视觉').replace('融合前KCF','原始KCF')
source+='''
comparison_rows=[]
for session,label in [('20261020_204102_kcf_only_composite_teach','10/20 20:41 仅KCF'),('20261021_positive20_teach_synthesis','10/21 15:23 自动卡尔曼'),('20261021_163204_sim01_kcf09_composite_teach','10/21 16:32 固定0.1/0.9')]:
    previous=pd.read_csv(OUT.parent/session/'metrics.csv')
    for key in ['TCP','raw_KCF']:
        a=previous[previous.trajectory.eq(key)].iloc[0]
        comparison_rows.append(dict(session=label,name=labels[key],n=int(a.n),mean_mm=float(a.mean_mm),max_mm=float(a.max_mm),rmse_mm=float(a.rmse_mm),within3=f"{float(a.within_3mm_fraction)*100:.2f}%"))
pd.DataFrame(comparison_rows).to_csv(OUT/'three_run_comparison.csv',index=False,encoding='utf-8-sig')
three_table=table(comparison_rows,[('session','会话'),('name','轨迹'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('within3','≤3 mm')])
body=f\'''<main><span class="badge">2026-10-20 · 20:41会话 · 仅KCF · 固定综合示教参考</span>
<h1>在线纠偏结果分析：仅KCF数据<br>与移动后综合原始示教的空间误差对比</h1>
<p class="meta">会话 {html.escape(r['session'])}｜采集 2026-10-20 20:41:02–20:53:25（北京时间）｜有效TCP 20:41:46–20:53:13｜出风口正20 mm｜长度单位mm</p>
<div class="actions"><a href="index.html">独立打开三维图</a><a href="axis_fluctuations.html">XYZ波动图</a><a href="metrics.csv">主统计表</a><a href="results.json">完整分析数据</a></div>
<div class="cards"><div class="card"><span>TCP平均误差</span><b>{tcp['mean_mm']:.3f} mm</b><span>RMSE {tcp['rmse_mm']:.3f} mm</span></div><div class="card"><span>KCF平均误差</span><b>{kcf['mean_mm']:.3f} mm</b><span>RMSE {kcf['rmse_mm']:.3f} mm</span></div><div class="card"><span>TCP最大误差</span><b>{tcp['max_mm']:.3f} mm</b><span>全部有效点在3 mm内</span></div><div class="card"><span>KCF最大误差</span><b>{kcf['max_mm']:.3f} mm</b><span>{kcf['above_3mm_count']}点超过3 mm</span></div></div>
<div class="lead"><b>结论：</b>以此前综合示教刚体移动后作参考，这组TCP与KCF平均误差均小于1 mm，RMSE约1.1 mm。TCP最大{tcp['max_mm']:.3f} mm，全部有效点在3 mm内；KCF最大{ kcf['max_mm']:.3f} mm，仍有局部峰值。相较10月21日16:32的0.1/0.9会话，这组平均误差和RMSE较大，但TCP最大误差更小。<b>不同指标呈现不同取舍，不能仅凭单次跨批次结果断言某种策略更好。</b></div>
<p class="note"><b>参考保持一致：</b>此前四组历史示教等权综合的原始示教圆，半径固定 <b>142.035339635 mm</b>。仅按本组原始KCF估计圆心和法线进行刚体移动；不使用本组TCP寻优，不重估综合半径。移动后是事后估计参考，尚非独立真实焊缝测量。</p>
<h2>1. 数据范围与实际运行模式</h2>
<p>读取用户指定目录，不替换为同日晚间其它会话。机器人记录{r['source_counts']['robot']:,}条、视觉诊断{r['source_counts']['diagnostics']:,}条。TCP筛选 online_running、CORFLAG=1、DETFLAG=1、S_START=1、S_HOLD=0、ALARM=0，保留{tcp['n']:,}点；KCF筛选 online_running、result_valid=1，保留{kcf['n']:,}点。保留全部有效点，无测量轨迹平滑或峰值剔除。</p>
<p><b>仅KCF得到日志与数值双重确认：</b>{kcf['n']:,}个有效在线视觉点的fusion_applied全部为0；before_fusion与final像素最大差为0，三维重投影最大差也为0。因此最终视觉与原始KCF完全重合，报告补充列出这条重复轨迹用于核查，不把它当成另一种策略。权重字段为空，不将缺失值解释为已记录的0或1。</p>
<p><b>“未启动在线ICP”的含义需要分开说明：</b>目录名是用户对本组测试的标注；session_summary中的enable_online_icp=true、{arc['batch_count']}批轨迹日志中{arc['applied']}批applied=1，记录的是<b>轨迹预测ICP</b>。当前代码OnlineCorrectionExceptionProcessor产生该消息；项目文档明确它与修改仿真工件位姿的<b>在线局部表面点云ICP</b>是不同链路。本报告保留用户对局部ICP关闭的标注，但不把这组数据写成“所有ICP均关闭”，也不根据轨迹批次推断发生过工件位姿写回。</p>
<h2>2. 同坐标系的四轨迹三维图</h2><p>统一到<b>变位机末节随动坐标系（材料系）</b>，逐点消除S1/S2旋转；视觉从工具TCP系加回Z=231 mm原点偏置。切换到工具TCP坐标系时只需所有圆心Z减231 mm，半径与距离不变。蓝：TCP，橙：原始KCF，灰色虚线：移动前综合示教，绿：移动后综合示教参考。</p>
<p>提供三维、XY、XZ、YZ视角、两组示教独立切换与Z显示倍率。默认1×等比例；倍率只影响显示，不改变任何坐标和误差。生成W点是此前综合参考的采样，不是本次新实测焊缝。</p>
<iframe src="index.html" title="四轨迹交互三维图" style="height:1040px" loading="eager"></iframe>
<h3>XYZ方向偏差随时间变化</h3><p>ΔX、ΔY、ΔZ分别为测量点减去参考圆最近点，用于观察毫米量级局部波动；不是TCP与KCF按同一时刻配对后的差值。最终视觉与KCF重合，默认隐藏重复轨迹。</p>
<iframe src="axis_fluctuations.html" title="XYZ波动图" style="height:990px" loading="lazy"></iframe>
<figure><img src="preview.png" alt="四轨迹等比例空间投影、三维距离和累计分布"><figcaption>静态总览保留真实坐标与等比例投影；细小Z波动可在交互图中放大显示。</figcaption></figure>
<h2>3. 综合示教来源、移动方式与圆参数</h2>{source_table}
<p>使用同一份已确认的四组综合示教，来源按XYZ几何去重，每组25%；历史W分别进行周向分箱、圆拟合后综合半径。未重新挑选来源或为这批数据调整尺寸。移动将综合圆心、法线对齐原始KCF拟合圆，保持尺度和半径；绕圆法线的相位不可辨识，采用最小旋转。</p>{circle_table}
<p>所有参数都在上述末节随动系。移动后中心、法线等于KCF拟合结果是对齐的定义，不能当成独立验证；TCP与KCF拟合圆仅用于描述，不替换它们的原始走势。</p>
<h2>4. 主误差结果与计算口径</h2>{metric_table}
<pre>c = 移动后综合圆心；n = 单位法线；R = 固定综合半径142.035339635 mm
h = (p-c)·n；ρ = ||(p-c)-h n||
e = √((ρ-R)²+h²)
平均 = mean(e)；最大 = max(e)；RMSE = √mean(e²)</pre>
<p>与前两份综合示教报告相同，采用点到<b>连续三维综合圆周</b>的解析最短距离，不混用到原单组示教折线、最近采样点或行号配对距离。均值按记录样本加权，不按弧长或时间加权；不评价工具姿态和沿缝时序滞后。3 mm用于描述局部偏离，不擅自定义为工艺合格线。</p>
<h2>5. 各轴、径向与离面误差</h2>{component_table}<p>有符号均值可能正负抵消，整体评价仍使用非负三维距离与RMSE。参考位姿来自KCF，因此整体位置和倾角误差可能被吸收，这些分量不是独立标定误差。</p>
<h3>前60秒与后续记录</h3>{window_table}<p>时间从保留轨迹的最早时间起算，不是会话启动时间。完整作业段统计见<a href="tcp_job_line_metrics.csv">TCP作业行指标</a>；分析中未删除起始段。</p>
<h2>6. KCF局部峰值</h2><p>TCP无点超过3 mm。原始KCF有{kcf['above_3mm_count']}点超过3 mm，占{100*(1-kcf['within_3mm_fraction']):.3f}%。事件按相邻记录均超过3 mm归组，相邻记录间隔超过200 ms则拆分；下表最多显示15段，最终视觉事件因与KCF相同不重复展示。</p>{event_table}
<p>最大KCF偏离在{peak['max_time']}，距离{peak['max_mm']:.3f} mm，径向分量{peak['radial_at_peak_mm']:.3f} mm、离面分量{peak['plane_at_peak_mm']:.3f} mm。轨迹只能确认局部偏离，不能单独判断检测跳点、真实焊缝局部形状或状态时效哪一个是原因。TCP未出现同等级峰值，也不能据此反推这些检测偏离已被某个指定机制完全抑制。</p>
<h2>7. 轨迹预测ICP证据与模式边界</h2><p>{arc['batch_count']}批记录均enabled=1、attempted=1，其中{arc['applied']}批应用，{arc['not_applied']}批未应用。按每批released_target点到本批raw_pending折线计算，已应用批次最大几何偏离{arc['released_departure_max_mm']:.3f} mm，各批平均偏离的中位数{arc['batch_mean_departure_median_mm']:.3f} mm。复算保留在<a href="trajectory_icp_effects.csv">轨迹批次效果表</a>。</p>
<p>该批次几何偏离既不是本报告的TCP焊缝误差，也不是局部表面配准残差；不以矩阵平移列直接当作真实工件位移。这里仅说明轨迹处理链路确实有应用记录，不能证明仿真工件发生SET_POSE写回。用户标注关闭的局部ICP与该记录并不必然冲突。</p>
<h2>8. 三组会话同参考口径对比</h2>{three_table}
<p>本组TCP平均/RMSE为0.937/1.083 mm，高于10月21日两组，但其最大2.968 mm低于0.1/0.9组的3.837 mm；本组KCF最大7.202 mm大于0.1/0.9组的3.419 mm。这反映平均偏离、极值和执行轨迹之间并不等价。</p>
<p>三组都固定同一综合半径，各自由本组KCF确定参考位姿。它们来自不同日期/时段、状态和观测，不是只改变一个参数的受控实验；不能把差异全部归因于融合策略或局部ICP开关。每组均保留全部有效点，不能用TCP较低峰值掩盖KCF异常。</p>
<h2>9. 参考敏感性与解释限制</h2><p>交错20°扇区留出后，KCF合并RMSE为{r['holdout']['pooled']['rmse_mm']:.3f} mm；有效KCF周向覆盖约{r['coverage_deg']:.2f}°。训练扇区只估计圆心/法线，半径仍固定。该检查降低同点重复拟合评价，但不能消除共享传感器和标定偏差。</p>{sens_table}
<p>留掉一组历史示教仅用于展示半径来源敏感性，不从中选择更好结果，主参考始终固定。没有本次工件移动后的独立实测焊缝，不能将事后对齐参考称为独立真值；当前标定能复算日志，也不证明历史标定无误。</p>
<h2>10. 复核结果与后续建议</h2><p>原始图像点到固定站系投影最大复算差{r['verification']['final_projection_max_mm']:.2e} mm，末节随动系日志复算最大差{r['verification']['positioner_replay_max_mm']:.2e} mm；原始KCF与最终视觉像素、三维坐标差均为0。数据哈希、独立标量几何、误差分量、事件计数及图表坐标分别验证，记录见下方附件。</p>
<p>优先回看20:45:08附近的KCF峰值图像与机器人状态；若要比较策略优劣，应在尽量相同工件位置和运行条件下重复采集，并分别统计TCP表现与检测点峰值。判断绝对精度仍需要独立焊缝或工件位姿测量。</p>
<div class="actions"><a href="TCP_circle_error_components.csv">TCP逐点误差</a><a href="raw_KCF_circle_error_components.csv">KCF逐点误差</a><a href="over_3mm_events.csv">峰值事件</a><a href="axis_residual_summary.csv">各轴统计</a><a href="three_run_comparison.csv">三组对比</a><a href="trajectory_icp_effects.csv">轨迹ICP批次</a><a href="input_hashes.json">输入哈希</a><a href="verification.json">数值复核</a><a href="browser_verification.json">浏览器检查</a></div>
<p>报告内含三维和XYZ交互图、离线Plotly依赖及静态预览。复现依次运行本目录analyze.py、build_report.py、verify.py。所有新增文件仅在本目录，原始采集和此前分析均不修改。浏览器采用无鼠标渲染及DOM事件检查；未进行手动拖拽、缩放和平移验收。</p>
<footer>沿用此前读取的<a href="prior_notion_workflow_snapshot.json">Notion数据分析与协作工作流快照</a>（2026-10-19/18），未声称本次核对在线最新版。用户指令：不使用鼠标、不删除文件；参考固定为此前综合原始示教，移动后对比。</footer></main>\'''
write('report.html',page('仅KCF在线纠偏｜20261020 204102｜移动后综合示教误差分析',body))
print(json.dumps(dict(report=str(OUT/'report.html'),windows=windows),ensure_ascii=False,indent=2))
'''
# Event output contains a duplicate final-vision series for audit, not for the human table.
source=source.replace("ev.sort_values('max_mm',ascending=False).head(15)", "ev[ev.trajectory.ne('final_vision')].sort_values('max_mm',ascending=False).head(15)")
(OUT/'build_report.py').write_text(source,encoding='utf-8')
print('Prepared report specific to the only-KCF session.')
