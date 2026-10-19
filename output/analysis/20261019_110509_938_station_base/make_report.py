"""Create a self-contained Chinese analysis report from verified derived results."""
from pathlib import Path
import base64
import html
import json

OUT = Path(__file__).resolve().parent
data = json.loads((OUT / 'results.json').read_text(encoding='utf-8'))
m = data['metrics']


def table(items):
    rows = []
    for label, key in items:
        v = m[key]
        rows.append(f'<tr><td>{label}</td><td>{v["n"]:,}</td>'
                    f'<td>{v["mean_mm"]:.3f}</td><td>{v["max_mm"]:.3f}</td>'
                    f'<td>{v["rmse_mm"]:.3f}</td><td>{v["p95_mm"]:.3f}</td></tr>')
    return '<table><thead><tr><th>查询点 → 参考轨迹</th><th>样本数</th><th>平均 / mm</th><th>最大 / mm</th><th>RMSE / mm</th><th>P95 / mm</th></tr></thead><tbody>' + ''.join(rows) + '</tbody></table>'


def image(name, caption):
    encoded = base64.b64encode((OUT / name).read_bytes()).decode()
    return f'<figure><img src="data:image/png;base64,{encoded}" alt="{html.escape(caption)}"><figcaption>{caption}</figcaption></figure>'


def xyz(values):
    return '（' + ', '.join(f'{v:.3f}' for v in values) + '）'


def link(name, title):
    return f'<a href="{name}">{title}</a>'


primary = table([
    ('TCP → 移动后的原始示教', 'TCP_to_relocated_teach'),
    ('KCF → 同一条移动后示教', 'KCF_to_relocated_teach'),
    ('TCP → 当前整圈原始 KCF 折线', 'TCP_to_KCF'),
    ('KCF → 当前整圈 TCP 折线（反向补充）', 'KCF_to_TCP'),
])
cv = table([
    ('TCP → 由另一组扇区定位的示教', 'TCP_to_relocated_teach_sector_holdout'),
    ('KCF → 由另一组扇区定位的示教', 'KCF_to_relocated_teach_sector_holdout'),
])
fit = table([
    ('KCF → 拟合圆（内部拟合残差）', 'KCF_to_fitted_circle'),
    ('TCP → 同一个拟合圆（辅助）', 'TCP_to_fitted_circle'),
])
raw = table([
    ('TCP → 未移动的原始示教', 'TCP_to_original_teach'),
    ('KCF → 未移动的原始示教', 'KCF_to_original_teach'),
])
circle_k = data['kcf_fitted_circle_balanced']
circle_w = data['teach_fitted_circle_balanced']
radial_difference = circle_w['radius_mm'] - circle_k['radius_mm']
tip = data['display_registration']['normal_alignment_deg']
a0, b0 = data['common_station_base_pose_deg']
html_text = f'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>2026-10-19 在线纠偏轨迹分析｜变位机固定基座</title>
<style>
:root{{color-scheme:light}}*{{box-sizing:border-box}}body{{margin:0;background:#f2f5f8;color:#182b3a;font:16px/1.8 'Microsoft YaHei',sans-serif}}
main{{max-width:1240px;margin:28px auto;background:white;padding:36px 42px;border-radius:14px}}h1{{font-size:29px;line-height:1.5;margin:0 0 10px}}
h2{{font-size:22px;margin-top:34px;color:#154465}}h3{{font-size:17px;margin-top:24px}}p{{margin:12px 0}}.meta{{color:#5b6d7c;font-size:14px}}
.note{{padding:15px 20px;background:#eaf3fa;border-left:5px solid #287cb2;border-radius:4px}}.limit{{background:#fff5df;border-left-color:#ca961c}}
table{{width:100%;border-collapse:collapse;margin:16px 0;font-size:14px}}td,th{{padding:10px 12px;border-bottom:1px solid #dce3e9;text-align:right}}
td:first-child,th:first-child{{text-align:left}}th{{background:#eaf1f6}}figure{{margin:24px 0}}img{{width:100%;height:auto;border:1px solid #e1e6eb}}
figcaption{{font-size:14px;color:#526777;margin-top:8px}}a{{color:#126b9c}}code,pre{{font-family:Consolas,monospace;background:#f2f5f8}}pre{{padding:14px;white-space:pre-wrap;font-size:13px}}
li{{margin:6px 0}}.files a{{display:inline-block;margin-right:20px}}@media print{{body{{background:white}}main{{padding:0;margin:0}}h2{{break-after:avoid}}figure,table{{break-inside:avoid}}}}
</style></head><body><main>
<h1>在线纠偏轨迹分析：先定位示教，再统一比较</h1>
<p class="meta">会话：20261019_110509_938｜日期：2026-10-19｜单位：mm｜不修改生产代码和原始数据</p>
<div class="note"><b>采用用户确定的基准：</b>用本轮 KCF 点拟合当前焊缝圆，通过圆心和平面定位原始示教轨迹，
将移动后的原始示教作为 TCP、KCF 的共同内部参考。只做刚体变换，保持示教半径和所有局部形状不变。</div>

<h2>1. 主要结果</h2>
{primary}
<p>TCP 相对移动后示教的平均三维距离为 <b>{m['TCP_to_relocated_teach']['mean_mm']:.3f} mm</b>，
KCF 为 <b>{m['KCF_to_relocated_teach']['mean_mm']:.3f} mm</b>。
TCP 与检测轨迹之间的平均几何距离为 <b>{m['TCP_to_KCF']['mean_mm']:.3f} mm</b>。
后一项说明两条轨迹彼此较贴合，但不能证明它们贴合独立测量的真实焊缝。</p>
<p>示教半径比 KCF 拟合圆大 <b>{radial_difference:.3f} mm</b>，与共同参考下约 1.7～1.8 mm 的平均距离处于同一量级。
因此这组距离包含参考半径差、实际轮廓差和高度波动，不能全部归因为在线控制跟随误差。
不能仅因 TCP 的平均值小于 KCF，就断言 TCP 比 KCF 更准确。</p>
<p>最大 TCP→移动后示教距离发生于 <b>{m['TCP_to_relocated_teach']['max_time']}</b>；
最大 KCF→移动后示教距离发生于 <b>{m['KCF_to_relocated_teach']['max_time']}</b>。
所有有效点都纳入评价，没有为了降低最大值而剔除尖峰。</p>
<div class="note limit">本报告的“误差”是<b>离线三维几何距离</b>，不是绝对定位精度、独立焊缝检测精度或沿缝时间滞后。
移动后示教的位置由 KCF 拟合决定，属于内部参考。</div>

<h2>2. 标准量如何建立，为什么这样比较合理</h2>
<ol>
<li>原始 KCF 记录先补回变位机工装 TCP 偏移 <code>(0,0,231) mm</code>，与示教轨迹统一到随动末节系。</li>
<li>按绕初估圆心的角度每 2° 分组，组内 XYZ 取中位代表点，避免慢速区域和高频噪声过度影响定位。
拟合采用 PCA 圆平面和带 Huber 权重的几何圆；权重仅用于拟合，最终误差仍使用全部原始点。</li>
<li>原始示教也拟合圆。仅将其圆心移到当前 KCF 拟合圆心，并以法线间的最小旋转对齐圆平面。</li>
<li><b>不缩放半径、不做局部拉伸、不用 TCP 拟合参考、不逐点调整示教来追随 KCF。</b>
原始示教的 1,368 个点整体使用同一个刚体变换，保留真实的模板尺寸差异。</li>
<li>TCP、KCF 分别到同一条移动后示教折线计算点到线段的最短三维距离，避免直接按不等长数据的行号配对。</li>
</ol>
<p>工件与示教轨迹都接近圆环。圆心与平面可用于定位，但<b>仅凭圆无法确定绕圆法线的唯一转角</b>。
本报告不额外拟合该转角，采用法线对齐的最小旋转；这是一种可复现约定，不是测得了真实安装偏航。</p>
<p><code>_W</code> 是稠密示教参考；<code>_T</code> 仅有 5 个运动控制点。
不将 5 个控制点直接连成折线来替代圆弧，否则会引入弦与圆弧之间的虚假误差。</p>

<h3>拟合圆与刚体移动参数</h3>
<table><thead><tr><th>量</th><th>原始示教圆</th><th>当前 KCF 拟合圆</th></tr></thead><tbody>
<tr><td>圆心 XYZ（统一外轴姿态下的固定基座，mm）</td><td>{xyz(data['teach_original_circle_center_common_station_base_mm'])}</td><td>{xyz(data['kcf_circle_center_common_station_base_mm'])}</td></tr>
<tr><td>半径（mm）</td><td>{circle_w['radius_mm']:.3f}</td><td>{circle_k['radius_mm']:.3f}</td></tr>
<tr><td>圆平面对齐最小转角</td><td colspan="2">{tip:.4f}°</td></tr>
<tr><td>尺度</td><td colspan="2">1.000000，保持不变</td></tr>
</tbody></table>
<p>上述圆心使用 S1={a0:.4f}°、S2={b0:.4f}° 的共同姿态。
完整变换矩阵见 {link('teach_to_current_common_station_base_transform.csv','固定基座变换矩阵')}；
输出模板见 {link('relocated_teach_common_station_base.csv','移动后示教固定基座坐标')}。
它不同于程序启动时仅用一个检测起点生成的平移参考，本报告未把那个启动参考当作当前标准量。</p>

<h2>3. 轨迹图</h2>
{image('01_trajectories_common_station_pose.png','图1：统一外轴姿态后的固定变位机基座表达。绿色是共同参考，蓝色为 TCP，橙色为原始 KCF，灰色是未移动原示教。Z 方向刻意放大；XY 轮廓贴合不能替代三维误差。')}
<p>图1只用已知外轴运动做姿态归一，没有拟合 TCP、没有旋转或平移 TCP 来压低误差。
它用于显示同一材料轮廓，<b>不表示机器人实际在固定基座中绕行整圈</b>。实际物理轨迹见图2。</p>
{image('02_native_station_base.png','图2：各自采样姿态下的固定基座轨迹。TCP 与前视 KCF 位于工件圆环的不同位置，因此物理轨迹只覆盖局部区域；绿色整圈参考以在线开始姿态展示。KCF 外轴由邻近 RobotState 插值近似重建。')}
{image('03_geometric_distances.png','图3：距离随在线时间变化。最上行为 TCP/KCF 之间的全周期几何贴合；中间是未定位示教的名义布局差异，不作性能指标；最下行使用用户指定的移动后示教共同参考。')}

<h2>4. 坐标系与指标定义</h2>
<p>固定变位机基座记 S，机器人基座记 R，随动末节记 L，工装 TCP 系记 C。
RBCALIB 安装参数为 t=(768.715,20.782,-511.246) mm，姿态=(-1.0631,-0.3026,0.0135)°。</p>
<pre>A = Rx(1.0631°) Ry(0.3026°) Rz(-0.0135°)
p_TCP,S = A (p_TCP,R - t)
B(s1,s2) = Ry(s1) Rz(s2)
p_KCF,L = p_KCF,C + (0,0,231)
p_TCP,L = Bᵀ p_TCP,S
p_示教移动,L = R_align p_原始示教,L + t_align
q_L = 移动后示教折线上离查询点最近的线段投影点
e_S = B(s1,s2) (p_L - q_L)
d = ||e_S||₂
平均误差 = mean(d)
最大误差 = max(d)
RMSE = sqrt(mean(d²))</pre>
<p>每一对查询点和参考对应点都放在<b>同一个外轴姿态的固定基座</b>后计算残差。
共同刚体旋转不改变距离，因此可在 L 中稳定找材料位置，再在 S 中表达残差。
这里的 RMSE 是三维距离 RMSE，不再除以 3；平均与 RMSE 均按有效原始样本加权。</p>
<p>同记录时刻 TCP 与 KCF 物理间距平均约 {data['contemporaneous_sensor_tcp_separation_not_error_mm']['mean_mm']:.2f} mm，
主要含前视间距，未把它当作纠偏误差。TCP→KCF 使用整圈检测折线，允许对到不同时间观测的同一材料位置。
最大 TCP→KCF 距离的查询时刻为 {m['TCP_to_KCF']['max_time']}，最近 KCF 段记录于 {m['TCP_to_KCF']['closest_kcf_record_time_at_max']}，
说明这项是全周期离线几何一致性，不是控制器当时的因果跟踪误差。</p>

<h2>5. 数据范围与不确定性</h2>
<ul>
<li>正式在线 TCP：{data['tcp_start']} 至 {data['tcp_end']}，7,807 点；要求 phase=online_running、corflag=detflag=1、s_start=1、s_hold=alarm=0。</li>
<li>正式在线 KCF：{data['kcf_start']} 至 {data['kcf_end']}，10,156 个有效点。未对最终评价数据作平滑或离群点删除。</li>
<li>原始记录共 9,421 条 TCP 状态、10,452 条检测结果。初始反扫、接近、状态切换边界及结束等待均未混入主统计。</li>
<li>排除等待主程序停止阶段的 1,097 条 TCP 返回运动。截图中的结束等待超时不参与本次轨迹指标。</li>
<li>日志记录的是写盘时刻，没有保存每个 KCF 点对应的曝光时刻、冻结外轴与帧号，故“各自采样姿态的 KCF 固定基座轨迹”只能近似重建。
主距离在相同材料位置及共同姿态下计算，不依赖把 KCF 写盘时刻当成精确曝光时刻。</li>
<li>KCF 与 TCP 共享机器人反馈和标定链，且共同参考由 KCF 定位；数据不具备独立测量真值条件。不能由此证明绝对焊接精度。</li>
<li>近圆环无法唯一识别绕法线转角；若需要每个示教控制点的真实角度对应关系，需独立特征、定位基准或已知工件位姿。</li>
</ul>

<h2>6. 辅助复核</h2>
<h3>圆形一致性，而非检测真值误差</h3>{fit}
<p>KCF→拟合圆的残差包含非圆形、非平面、视觉与标定误差等因素，不能单独解释为随机检测噪声。</p>
<h3>扇区留出敏感性复核</h3>
<p>按极角划为 8 个连续 45° 扇区，交替 4 扇区定位、4 扇区评价，再交换；未使用随机或奇偶帧划分。
TCP 不参与拟合。此检查用于看定位结果对局部观测的敏感性，仍不是独立真值验证。</p>{cv}
<h3>未移动原始示教的对照值</h3>{raw}
<p>这些数值包含新旧工件位置差异，仅供说明为何应先定位示教；不作为纠偏效果结论。</p>

<h2>7. 可复查数据与审查分工</h2>
<p class="files">{link('metrics.csv','全部指标 CSV')}{link('tcp_coordinates.csv','TCP 坐标')}
{link('kcf_coordinates.csv','KCF 坐标与近似外轴')}{link('TCP_to_relocated_teach_distances.csv','TCP 到共同参考逐点距离')}
{link('KCF_to_relocated_teach_distances.csv','KCF 到共同参考逐点距离')}{link('TCP_to_KCF_distances.csv','TCP 到 KCF 逐点对应')}
{link('results.json','完整参数、口径及输入 SHA256')}{link('analyze.py','复现分析程序')}</p>
<p>子 agent A 核查坐标、单位与记录时序；子 agent B 核查比较口径，并独立复算 TCP/KCF 距离与圆定位方案。
主 agent 完成最终稳健圆拟合、共同参考构建、全部指标、图表与本报告。
独立复核结果与最终结果接近；最终数值以本目录明确的角度分组与稳健拟合方法为准。</p>
<p class="meta">输入数据与 SHA256 已记录于 results.json。未修改生产代码、原始会话记录、示教或标定文件；本目录仅为派生分析结果。</p>
</main></body></html>'''
(OUT / 'report.html').write_text(html_text, encoding='utf-8')
print(OUT / 'report.html')
