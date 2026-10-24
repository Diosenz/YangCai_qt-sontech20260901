"""Use the existing visual template with two measurable series and honest provenance."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261020_202202_auto_kalman_composite_teach'
source=(TEMPLATE/'build_report.py').read_text(encoding='utf-8').split('comparison_rows=[]',1)[0]
source=source.replace("labels={'TCP':'TCP 实际轨迹','raw_KCF':'KCF 原始检测','final_vision':'卡尔曼融合后视觉'}", "labels={'TCP':'TCP 实际轨迹','recorded_vision':'记录视觉输出（融合状态未知）'}")
source=source.replace("colors={'TCP':'#0072b2','raw_KCF':'#d55e00','final_vision':'#8956b1'}", "colors={'TCP':'#0072b2','recorded_vision':'#d55e00'}")
source=source.replace("['TCP','raw_KCF','final_vision']", "['TCP','recorded_vision']").replace('raw_KCF','recorded_vision')
source=source.replace("['original-mode',3,5],['relocated-mode',4,6]", "['original-mode',2,4],['relocated-mode',3,5]")
source=source.replace('10月20日20:22','10月19日21:12').replace('自动卡尔曼','CFKONTEST3五个P点')
source=source.replace('KCF 原始检测','记录视觉输出').replace('原始KCF','记录视觉').replace('KCF融合前','记录视觉')
source=source.replace('；紫色卡尔曼融合后视觉可点击图例打开。','。').replace('图例可显示卡尔曼融合后视觉；','')
source='\n'.join(line for line in source.splitlines() if not line.startswith('comparison_table=') and not line.startswith("tcp=r['metrics']"))+'\n'
source+='''
tcp=r['metrics']['TCP'];vision=r['metrics']['recorded_vision'];peak=ev.sort_values('max_mm',ascending=False).iloc[0]
comparison_rows=[];report_hashes={}
for directory,label in [('20261019_211208_five_p_composite_teach','10/19 21:12 五个P点'),('20261020_202202_auto_kalman_composite_teach','10/20 20:22 自动卡尔曼'),('20261020_204102_kcf_only_composite_teach','10/20 20:41 仅KCF'),('20261021_positive20_teach_synthesis','10/21 15:23 自动卡尔曼'),('20261021_163204_sim01_kcf09_composite_teach','10/21 16:32 固定0.1/0.9')]:
    path=OUT.parent/directory/'metrics.csv';previous=pd.read_csv(path);report_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for key,description in [('TCP','TCP'),('recorded_vision' if directory==OUT.name else 'final_vision','记录视觉（融合状态未知）' if directory==OUT.name else '最终视觉')]:
        a=previous[previous.trajectory.eq(key)].iloc[0]
        comparison_rows.append(dict(session=label,name=description,n=int(a.n),mean_mm=float(a.mean_mm),max_mm=float(a.max_mm),rmse_mm=float(a.rmse_mm),within3=f"{a.within_3mm_fraction*100:.2f}%"))
pd.DataFrame(comparison_rows).to_csv(OUT/'five_run_output_comparison.csv',index=False,encoding='utf-8-sig')
comparison_table=table(comparison_rows,[('session','会话'),('name','轨迹类型'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('within3','≤3 mm')])
body=f\'''<main><span class="badge">2026-10-19 · 21:12会话 · CFKONTEST3五个P点 · 综合示教参考</span>
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
<footer>沿用已有<a href="prior_notion_workflow_snapshot.json">Notion数据分析与协作工作流快照</a>（2026-10-19/18），未重新声称读取在线最新版。不使用鼠标、不删除文件、不修改业务源码；参考按用户确认固定为此前综合原始示教。</footer></main>\'''
write('report.html',page('CFKONTEST3五个P点｜20261019 211208｜移动后综合示教误差分析',body))
report_hashes[str(Path(__file__))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write('report_input_hashes.json',json.dumps(report_hashes,ensure_ascii=False,indent=2))
print(json.dumps(dict(report=str(OUT/'report.html'),windows=windows),ensure_ascii=False,indent=2))
'''
(OUT/'build_report.py').write_text(source,encoding='utf-8')
print('Prepared five-P report with recorded-vision semantics and timeout evidence.')
