"""Build a readable report from saved replay results; does not read devices."""
from pathlib import Path
import html,json
OUT=Path(__file__).resolve().parent
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
f=json.loads((OUT/'follow_diagnostics.json').read_text(encoding='utf-8'))
rows=''.join(f'<tr><td>{label}</td><td>{r["metrics"][key]["mean_mm"]:.3f}</td><td>{r["metrics"][key]["median_mm"]:.3f}</td><td>{r["metrics"][key]["p95_mm"]:.3f}</td><td>{r["metrics"][key]["max_mm"]:.3f}</td></tr>' for label,key in [('TCP → 事后定位示教折线','TCP_to_relocated_teach'),('TCP → 全程融合前KCF折线','TCP_to_raw_KCF_polyline'),('融合前KCF → 事后定位示教折线','raw_KCF_to_relocated_teach'),('TCP → 原始示教折线','TCP_to_original_teach'),('TCP → 启动首点平移后的示教折线','TCP_to_startup_aligned_teach')])
jobrows=''.join(f'<tr><td>{x["job_line"]}</td><td>{x["start"]}～{x["end"]}</td><td>{x["n"]}</td><td>{x["mean_mm"]:.3f}</td><td>{x["p95_mm"]:.3f}</td><td>{x["max_mm"]:.3f}</td></tr>' for x in r['job_line_metrics'])
report='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>在线纠偏最新复测｜证据与排查结论</title><style>
body{max-width:1100px;margin:32px auto;padding:0 25px;font:16px/1.8 system-ui,"Microsoft YaHei",sans-serif;color:#203345}h1{font-size:28px}h2{margin-top:30px;font-size:22px}h3{font-size:18px}table{border-collapse:collapse;width:100%;margin:15px 0}th,td{border-bottom:1px solid #d9e1e6;padding:9px;text-align:left}th{background:#f2f6f8}a{color:#0072b2}img{max-width:100%;height:auto}.note{padding:12px 18px;border-left:4px solid #d55e00;background:#fff8ef}code{overflow-wrap:anywhere}li{margin:8px 0}</style></head><body>
<h1>在线纠偏最新复测：修正确实发生，但局部通过没有代表整圈准确</h1>
<p>数据：2026-10-21 11:42:11～11:51:47（北京时间），会话 <code>20261021_114211_739</code>。主 agent 汇总三个子 agent 的只读调查，并独立复算关键数值。已参考用户指定 Notion 协作流程与数据展示偏好。</p>
<p><a href="index.html">打开四曲线交互三维图</a> · <a href="metrics.csv">轨迹指标</a> · <a href="native_2d_3d_consistency.csv">384帧二维/三维逐帧核查</a> · <a href="follow_geometry.csv">跟随外推诊断</a> · <a href="input_hashes.json">输入哈希</a></p>
<div class="note"><b>主结论：</b>最优先核对 RobotSim 二维/三维观测与真实标定之间的不一致，以及启动起点相位与整段首点平移假设。工件局部回写只成功一次，附近三帧通过后其他方位仍明显失配；在线轨迹 ICP 和短线外推又可能放大目标方向波动。现有证据不支持先放宽门槛或仅增大纠偏强度。</div>
<h2>1. 结果量化与口径</h2>
<p>选择在线运行且纠偏/检测有效、机器人启动、未暂停、无报警的 TCP 5,838 点；在线有效融合前 KCF 7,889 点；原始示教 1,368 点。统一回到变位机末节/材料坐标系，消除逐帧外轴旋转。真实像素重建使用当帧机器人姿态，补回工装 TCP 的 231 mm。图中再统一到一个固定外轴姿态显示。</p>
<p>绿色参考：用融合前 KCF 整圈数据估计圆心与平面法线，只对原始示教施加刚体变换，保持原始半径，不添加任意绕法线相位。距离严格计算点到全部原始示教折线线段的最短三维距离；不是到最近采样点或拟合圆的距离。TCP/KCF原始走势均保留，未平滑或剔除异常点。</p>
<table><thead><tr><th>口径</th><th>均值/mm</th><th>中位/mm</th><th>P95/mm</th><th>最大/mm</th></tr></thead><tbody>METRICS</tbody></table>
<p><b>解释边界：</b>绿色是事后诊断参考，依赖本轮 KCF，不能替代独立焊缝真值，也不是控制器每一时刻实际采用的目标。TCP 到整圈 KCF 折线为几何距离，不是按时间配对的动态跟踪误差。原始圆参考半径143.717 mm，KCF描述性拟合半径141.347 mm；约2.37 mm差异仍可能来自轨迹、检测、标定或工件模型，不能直接认定工件尺寸有误。</p>
<img src="preview.png" alt="TCP偏差、图像偏差及四类轨迹概览">
<table><thead><tr><th>机器人程序行</th><th>时间</th><th>TCP点数</th><th>均值/mm</th><th>P95/mm</th><th>最大/mm</th></tr></thead><tbody>JOBS</tbody></table>
<p>明显大偏差集中于前两段，最大15.174 mm发生在11:45:20.615。后两段均值约2.45/2.20 mm。写回前仅20.8秒、后面覆盖不同方位，不能将前后均值直接解释为写回的因果效果。上午10:35一轮在134秒时提前停止，也不能与这轮整体均值直接比较。</p>
<h2>2. 已确认：RobotSim二维与三维先验经当前标定不自洽</h2>
<p>按 frame_id 交叉384个原始配准采样：融合前KCF像素经当前 SENSORPARA、RBCALIB 和当帧姿态重建，与归档真实三维点最大差仅1.21×10⁻⁷ mm，证明重放算法与本轮生产投影口径一致。用同一算法重建 RobotSim 的二维焊点，却与同帧原生三维焊点平均相差10.550 mm、中位10.403 mm、最大11.955 mm。</p>
<p>例如frame3207，RobotSim二维(526,600)重建为(-77.497874,-120.736494,18.832777) mm，原生三维是(-74.333627,-115.674640,26.293674) mm，相差9.555 mm。该比较已统一父坐标系与单位，不涉及界面缩放。</p>
<p>回写前frame3207的raw KCF到sim距离38.35 px；回写后用于验收的frame3276约133.97 px，但原生三维点的验收误差却只有1.09 mm。两帧外轴角度并非完全相同，不能把像素前后差当严格同姿态实验；同一帧二维/三维的10 mm不一致则可直接复现。</p>
<p><b>已确认不一致，具体责任项尚待核查：</b>RobotSim传感器1013的图像尺寸、焦距/像元/主点/畸变、相机外参、激光平面、TCP以及输出参考对象1063的语义。不能只凭此结果选择修改某份标定文件。</p>
<p>代码：DigitalTwin/src/RobotSimDigitalTwinService.cpp:310、804读取同一GET_WELD_SEAM_PHOTO点对；Vision/src/geometry/WeldImageToSpaceProjector.cpp:184、195、205、253实施真实投影；UI/src/RuntimeWorkerAdapters.cpp:1460投影融合前KCF作为独立验收点。ROI和同帧叠加缩放未发现能解释该冲突的错误。</p>
<h2>3. 已确认：工件只成功回写一次，验证范围很局部</h2>
<p>48次候选归档，最后reason分类为37次独立焊点否决、7次覆盖/跨度否决、2次幅度验收未过、2次局部通过。各门控可覆盖reason，因此这不是互斥根因计数。</p>
<ul><li>frame3207唯一成功写回：Rz=-5.873538°，t=(9.461327,-13.443195,-4.379898) mm；表面单向平均最近邻残差5.588→1.665 mm，窗口独立缝点最坏误差7.035→1.191 mm。</li><li>11:43:16附近3个新帧3276/3278/3280，跨度131 ms，原生三维缝点误差1.091/1.028/1.027 mm，随后晋升可信。</li><li>后续360个归档独立观测的同帧真实/虚拟缝点误差：均值8.151、中位9.057、P95 13.562、最大17.292 mm。这个口径独立于事后示教轨迹距离。</li><li>frame11481第二次局部通过，但累计旋转12.9225°超过10°预算，发送前拒绝；11:47:48自动工件更新停用，在线轨迹纠偏继续。它不是第二次成功修正。</li></ul>
<p>每次8帧局部窗口仅覆盖约2～3秒，采用X/Y/Z/Rz四自由度，求解后清空。真实光条中段覆盖较窄，例如frame8133真实Z跨度21.33 mm、虚拟78.47 mm。根据当前代码及数据推测，局部对应/表面覆盖不足与旋转平移耦合可能贡献失配，需先检查真实光条分支，不宜放宽覆盖或独立点门槛。</p>
<p>后验旧位姿“反事实”由逆变换当前虚拟缝点得到；若缝点是激光平面与工件曲线交点，这不等价于以旧模型在同一激光平面重新求交，不能当严格未修正对照。绝对当前误差仍有价值。代码：UI/include/OnlineLocalRegistrationVerification.h:23、44；UI/src/OnlineLocalRegistrationWorker.cpp:389、399；UI/src/RuntimeLocalRegistration.cpp:475、622。关键点位移预算目前也只检查当前一个缝点，不能代表整件最大位移。</p>
<h2>4. 已确认：本轮起点数据与“整段首点平移”假设不相容</h2>
<p>末扫点(-125.684837,-64.914331,251.160994) mm被当作_W/_T段首点，整段平移(8.312163,-108.999331,-2.395006) mm。它距原_W首点109.342 mm，而距原_W全折线仅6.650 mm，位于零基169段附近；反扫方向与该处反向切线余弦0.987。数据强烈提示起点相位不同。</p>
<p>TCP到启动平移后参考折线均值63.955 mm、最大97.690 mm。在线ICP每批仅逆变换模型并释放本批局部目标，后续相位重定位只改游标，未持久修正这个底层参考。</p>
<p><b>这不是已确认的实现违约：</b>docs/online_correction_state_machine.md:104明确约定末扫点对齐段首、仅平移、当前不执行最近点匹配。需要核对现场扫起点、_W/_T物理相位和模式约定，再决定是否调整策略；不能静默改成最近点起步。代码Control/src/OnlineCorrectionTrajectoryPlanner.cpp:465、499。</p>
<h2>5. 在线跟随链的附加问题</h2>
<p>另一条Control轨迹ICP共449批、420批应用；RMS中位0.257 mm，但旋转角中位68.165°、相邻变化中位45.502°。7～56个短弧点与1368点模型做无对应最近邻配准，在近圆、短弧和噪声下容易病态或相位多解。第1批PCA仍满秩，不能宣称短弧数学上必然不可观察。变换方向和+231补偿已核对，未见反用或重复补偿。</p>
<p>大变换不表示机器人或工件真的转动这些角度：此链只产生局部释放目标，不回写RobotSim工件。当前接受条件主要检查有限值、合法刚体与RMS改善，缺少连续性和物理变化约束。代码Control/src/OnlineCorrectionExceptionProcessor.cpp:213、225、243。</p>
<p>7,591个跟随目标全部使用first→third短线向后外推(t&lt;0)：线长中位1.998 mm，当前位置到first最大41.135 mm，内部点到目标直线残差P95 10.061 mm、最大25.287 mm。短线切向波动容易被外推放大；该内部残差不能直接叫真实焊缝精度。代码Control/src/OnlineCorrectionPathFollower.cpp:71、302。</p>
<p>跟随器固定0.1 mm/s基础纠偏速度，用归一化误差方向乘配置周期4.3 ms，再作PID与量化；输出通常只有1～3 µm。误差很大并不会按误差幅值直接增强输出。命令生成7,592次，P回执2,897次均成功；生成间隔中位61 ms，回执中位175 ms，通信worker采用latest覆盖。接收端是逐包执行一次还是内部周期复用最新增量尚不明确，不能把包数差直接判为纠偏量丢失，也不能据此盲目增益。代码PathFollower.cpp:350、379；UI/src/RuntimeWorkerAdapters.cpp:928。</p>
<h2>6. 运行证据的限制与已排除方向</h2>
<ul><li>11:44:14.344之后，7,104帧融合实际未执行，kalman_seeded也为0。根据当前代码推测可能运行时切换到KcfOnly，但无模式变更日志，不能断言操作者行为。这轮不是可确认的全程固定融合配置对照。</li><li>全部8,097帧result_valid=1仅表示检测/投影链有效，不是精度验收；没有充分证据认定KCF全程锁错。已保存图片只覆盖前30对、约11:42:27～11:43:17，后续大偏差阶段缺原图。</li><li>本轮前段平移上限20 mm，到frame7957才变为30 mm；不能用当前INI的30 mm解释整轮。</li><li>会话以reset结束，没有完整终态验收。源代码存在用户未提交修改；当前可执行文件时间为11:39:23，本报告依据运行记录解释实测，代码用于说明当前机制，未重编译确认全部二进制对应关系。</li><li>已复算真实投影与归档一致、ICP方向正确、+231仅加一次；P回执全部成功。暂不支持把问题归咎于截图缩放、单纯网络错误或TCP补偿重复。</li></ul>
<h2>7. 建议下一轮先做的验证</h2>
<ol><li><b>先固定几何链：</b>在固定机器人/外轴姿态、固定工件下保存原图、RobotSim原始2D+3D点对、相机与激光面参数、参考对象链。将原生3D按真实标定正投影到图像，并检查激光平面约束。区分像素投影、三维参考坐标和真实线激光模型，先定位10 mm不一致来源。</li><li><b>核对启动相位：</b>记录反扫终点、_W/_T首点、最近段索引、方向与外轴。明确末扫点是否应与文件段首具有同一物理相位；这是现有策略前提核查，尚未授权改为最近点起步。</li><li><b>固定输入离线回放：</b>对比原始KCF目标、当前轨迹ICP目标、加连续性/相位约束的候选目标；同时比较当前TCP处的局部切向与外推距离，避免只比较拟合RMS。涉及Control/Vision实现需与模块负责人协调。</li><li><b>最后再调闭环：</b>确认P命令接收端的增量/保持语义与真实周期，再评估增益。用多个不同方位的观测验收工件姿态，覆盖整个运行期保存图片和模式变化；达到用户认可的几何和安全阈值后才考虑写回策略调整。</li></ol>
<h2>交付与验证</h2>
<p>新增内容仅在本分析目录。未修改业务源码、标定、现场配置或原始复测数据，未删除文件，未使用Computer Use或鼠标，未触发机器人动作或构建。数值重放断言及输入哈希复核通过；图已实际读取检查。交互图复用既有三维模板并默认显示实际示教点与连线，提供3D/XY/XZ/YZ、独立示教切换与Z倍率。用户禁止Computer Use，因此未做真实浏览器旋转、缩放或显示验收；仅完成脚本逻辑核查。</p>
<p>复现入口：<a href="analyze.py">analyze.py</a>、<a href="build_report.py">build_report.py</a>；已有依赖路径和版本由input_hashes记录，数值重放依赖numpy/pandas/Pillow。三维图已内嵌Plotly，可离线重新打开。<a href="results.json">完整轨迹结果</a> · <a href="local_consistency.json">局部一致性结果</a> · <a href="follow_diagnostics.json">跟随统计</a> · <a href="verification.json">验证记录</a></p>
<p>工作方法参考：<a href="https://app.notion.com/p/3fdaa04fa8a9819195d2dc0eb65ce630">AI协作工作流</a>、<a href="https://app.notion.com/p/3faaa04fa8a981d1b419d2fc27a13cb4">修bug专项流程</a>、<a href="https://app.notion.com/p/3feaa04fa8a9816c9f51c32fad802a46">数据展示偏好</a>。仅参考相关方法，未修改Notion。</p>
</body></html>'''.replace('METRICS',rows).replace('JOBS',jobrows)
(OUT/'report.html').write_text(report,encoding='utf-8')
print('Report created:',OUT/'report.html')
