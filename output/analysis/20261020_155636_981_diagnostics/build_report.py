"""Reproduce a read-only analysis of the 2026-10-20 online correction run.
Requires numpy, pandas and Pillow. Plotly JS is reused from the existing local analysis.
Never changes source records or production code; outputs stay beside this script.
"""
from pathlib import Path
import hashlib
import html
import json
import shutil
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
SESSION = ROOT / 'output/online_correction/20261020_155636_981_增加有过程记录的测试'

def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def rotation(angles):
    a,b,c=np.radians(angles); return rz(c)@ry(b)@rx(a)
def load(name):
    frame=pd.read_csv(SESSION/name, sep='\t')
    assert not frame.columns.str.startswith('Unnamed:').any(), name
    return frame

d=load('vision_diagnostics.tsv'); d['sample_index']=np.arange(len(d))+1
assert len(d)==2124 and not d.frame_id.duplicated().any()
assert d.session_id.nunique()==1 and d.camera_timestamp_ms.is_monotonic_increasing
assert (d.fusion_jump_rejected==0).all() and (d.correlation_jump_rejected==0).all()
manifest=pd.read_csv(SESSION/'vision_images/image_manifest.tsv',sep='\t')
assert len(manifest)==30 and (manifest.status=='saved').all()
for row in manifest.itertuples():
    paths=[SESSION/'vision_images'/f'{row.stem}_{kind}.png' for kind in ('source','enhanced')]
    assert sum(p.stat().st_size for p in paths)==row.encoded_bytes
    for path in paths:
        with Image.open(path) as im: im.verify()

sensor=ROOT/'config/calibration/SENSORPARA.txt'; rb=ROOT/'config/calibration/RBCALIB.DAT'
vals={l.split()[0]:float(l.split()[1]) for l in sensor.read_text().splitlines()
      if len(l.split())==2 and not l.startswith(('More','2026'))}
cr=rotation([vals[f'pose_CinT[{i}]/deg'] for i in (3,4,5)])
ct=np.array([vals[f'pose_CinT[{i}]/m']*1000 for i in (0,1,2)])
srang=next(l.split()[1] for l in rb.read_text().splitlines() if l.startswith('////SRANG'))
v=np.array([float(x) for x in srang.split(',')])
sr=rx(np.radians(-v[3]))@ry(np.radians(-v[4]))@rz(np.radians(-v[5])); st=-sr@v[:3]

def project(x,y,state):
    """Same distortion, ray/laser-plane intersection and rigid chain as production.
    Result: fixed positioner base frame, mm. State uses logged frozen robot pose.
    """
    uv=(np.array([x,y])-np.array([vals['cx'],vals['cy']]))*np.array([vals['sx/m'],vals['sy/m']])
    uv/=1+vals['Kappa']*(uv@uv)
    ray=np.r_[uv/vals['f/m'],1]
    cam=-1000/(np.array([vals['a'],vals['b'],vals['c']])@ray)*ray
    rt=rotation(state[['robot_rx_deg','robot_ry_deg','robot_rz_deg']].to_numpy(dtype=float))
    rp=rt@(cr@cam+ct)+state[['robot_x_mm','robot_y_mm','robot_z_mm']].to_numpy(dtype=float)
    return sr@rp+st

calc=np.array([project(r.final_x_px,r.final_y_px,r) for _,r in d.iterrows()])
logged=d[['station_base_x_mm','station_base_y_mm','station_base_z_mm']].to_numpy()
residual=np.linalg.norm(calc-logged,axis=1)
assert residual.max()<2e-6, 'Local calibration does not reproduce run; do not decompose using it.'
d['online_adjacent']=d.phase.eq('online_running')&d.phase.shift().eq('online_running')
d['t_s']=(d.camera_timestamp_ms-d.camera_timestamp_ms.iloc[0])/1000
d['time_cn']=pd.to_datetime(d.camera_timestamp_ms,unit='ms',utc=True).dt.tz_convert('Asia/Shanghai').dt.strftime('%H:%M:%S.%f').str[:-3]
for axis in 'xy':
    c=d[f'before_fusion_{axis}_px']; s=d[f'sim_{axis}_px']; f=d[f'final_{axis}_px']
    k=d[f'kcf_weight_{axis}'].fillna(1) # All non-fused frames are Kalman seeding; no rejection this run.
    d[f'dkcf_{axis}']=c.diff();d[f'dsim_{axis}']=s.diff();d[f'dfinal_{axis}']=f.diff()
    d[f'fusion_offset_{axis}']=f-c;d[f'dfusion_{axis}']=(f-c).diff()
    # Exact endpoint decomposition using previous K, then changing K at current C and S.
    d[f'raw_term_{axis}']=k.shift()*c.diff()
    d[f'prior_term_{axis}']=(1-k.shift())*s.diff()
    d[f'weight_term_{axis}']=k.diff()*(c-s)
    error=d[f'dfinal_{axis}']-sum(d[f'{term}_term_{axis}'] for term in ('raw','prior','weight'))
    assert error.dropna().abs().max()<.0001
    d[f'dominant_{axis}']=np.where(d[f'weight_term_{axis}'].abs()>d[f'raw_term_{axis}'].abs(),'weight_change','kcf_change')
d['dz_mm']=d.station_base_z_mm.diff(); d['pixel_dz_mm']=np.nan;d['pose_dz_mm']=np.nan
d['pixel_u_dz_mm']=np.nan;d['pixel_v_dz_mm']=np.nan
for i in range(1,len(d)):
    a,b=d.iloc[i-1],d.iloc[i]
    q=project(b.final_x_px,b.final_y_px,a)
    q_u=project(b.final_x_px,a.final_y_px,a)
    d.loc[i,'pixel_u_dz_mm']=q_u[2]-calc[i-1,2]
    d.loc[i,'pixel_v_dz_mm']=q[2]-q_u[2]
    d.loc[i,'pixel_dz_mm']=q[2]-calc[i-1,2]
    d.loc[i,'pose_dz_mm']=calc[i,2]-q[2]
assert np.nanmax(abs(d.dz_mm-d.pixel_dz_mm-d.pose_dz_mm))<2e-6
on=d[d.phase.eq('online_running')].copy(); adj=d[d.online_adjacent].copy()
assert len(on)==1749 and len(adj)==1748
xjump=adj[adj.dfinal_x.abs()>5];yjump=adj[adj.dfinal_y.abs()>5]
assert len(xjump)==124 and len(yjump)==15
assert d[d.template_reinitialized.eq(1)&d.phase.eq('online_running')].index.to_series().diff().dropna().eq(51).all()
d.to_csv(OUT/'diagnostics_enriched.csv',index=False,encoding='utf-8-sig')
events=adj[(adj.dfinal_x.abs()>5)|(adj.dfinal_y.abs()>5)].copy()
events['screening_definition']='within online phase; |du|>5 px or |dv|>5 px; not ground-truth error'
events.to_csv(OUT/'jump_events.csv',index=False,encoding='utf-8-sig')

b=load('online_icp_batches.txt');pt=load('online_icp_points.txt');follow=load('online_follow_targets.txt')
raw_lookup={tuple(np.round([r.positioner_tcp_x_mm,r.positioner_tcp_y_mm,r.positioner_tcp_z_mm+231],6)):int(r.frame_id)
            for r in d.itertuples()}
raw=pt[pt.kind.eq('raw_pending')].copy()
raw['source_frame_id']=[raw_lookup.get(tuple(np.round(q,6))) for q in raw[['x_mm','y_mm','z_mm']].to_numpy()]
assert raw.source_frame_id.notna().all()
raw.to_csv(OUT/'icp_raw_to_frame.csv',index=False,encoding='utf-8-sig')
rows=[]
for bi,g in pt.groupby('batch_index',sort=False):
    q=g[g.kind.eq('raw_pending')][['x_mm','y_mm','z_mm']].to_numpy()
    r=g[g.kind.eq('released_target')][['x_mm','y_mm','z_mm']].to_numpy()
    if len(q)<2 or len(r)==0:continue
    a=q[:-1];vec=q[1:]-a;delta=r[:,None]-a
    frac=np.clip(np.einsum('ijk,jk->ij',delta,vec)/np.maximum((vec*vec).sum(1),1e-24),0,1)
    distance=np.sqrt(((delta-frac[:,:,None]*vec)**2).sum(2)).min(1)
    rows.append([bi,len(q),len(r),distance.mean(),distance.max()])
bs=pd.DataFrame(rows,columns=['batch_index','raw_count','released_count','mean_to_raw_polyline_mm','max_to_raw_polyline_mm'])
bs=bs.merge(b[['batch_index','timestamp_ms','applied','rms_mm','message']],on='batch_index')
bs.to_csv(OUT/'icp_batch_effects.csv',index=False,encoding='utf-8-sig')

# Use per-row external axes, rather than interpolating axes for diagnostic vision points.
def to_material(points,s1,s2):
    out=[]
    for q,a,z in zip(points,s1,s2): out.append(rz(np.radians(-z))@ry(np.radians(-a))@q)
    return np.asarray(out)
robot=load('actual_robot_path.txt');r=robot[robot.phase.eq('online_running')].copy()
fixed=r[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st
rq=to_material(fixed,r.s1_deg.to_numpy(),r.s2_deg.to_numpy())
vq=on[['positioner_tcp_x_mm','positioner_tcp_y_mm','positioner_tcp_z_mm']].to_numpy()+[0,0,231]
initial=load('initial_correction.txt');corrected=initial[initial.event.eq('corrected_path_point')][['converted_x_mm','converted_y_mm','converted_z_mm']].to_numpy()
work=ROOT/'data/WorkPiece/20260404CHUFENGKOU1_W.txt';original=np.loadtxt(work)[:,:3]
pd.DataFrame(rq,columns=['x_mm','y_mm','z_mm']).to_csv(OUT/'tcp_material_frame.csv',index=False)
pd.DataFrame(vq,columns=['x_mm','y_mm','z_mm']).to_csv(OUT/'vision_material_frame.csv',index=False)

metrics={'session':SESSION.name,'completed':True,'diagnostic_count':len(d),'online_frames':len(on),'online_adjacent_pairs':len(adj),
 'screening_threshold_px':5,'x_events':len(xjump),'y_events':len(yjump),'max_abs_dx_px':float(adj.dfinal_x.abs().max()),
 'max_abs_dy_px':float(adj.dfinal_y.abs().max()),'max_abs_dz_mm':float(adj.dz_mm.abs().max()),
 'x_weight_dominant_events':int(xjump.dominant_x.eq('weight_change').sum()),
 'x_reinit_events':int(xjump.template_reinitialized.sum()),'online_reinit_opportunities':int(adj.template_reinitialized.sum()),
 'y_reinit_events':int(yjump.template_reinitialized.sum()),'projection_max_reproduction_error_mm':float(residual.max()),
 'gain_median':float(on.kcf_weight_y.median()),'image_pairs':len(manifest),'image_bytes':int(manifest.encoded_bytes.sum()),
 'icp_applied':int(b.applied.sum()),'icp_batches':len(b),'max_released_to_raw_polyline_mm':float(bs.max_to_raw_polyline_mm.max()),
 'max_command_um':follow[['dx_um','dy_um','dz_um']].abs().max().to_dict(),
 'image_last_camera_time':d[d.image_request_status.eq('queued')].time_cn.iloc[-1]}
inputs=[p for p in SESSION.rglob('*') if p.is_file()]+[sensor,rb,work,ROOT/'Vision/src/track/KcfTracker.cpp',ROOT/'Vision/src/geometry/WeldProjectionContextBuilder.cpp',ROOT/'Vision/src/geometry/WeldImageToSpaceProjector.cpp',ROOT/'Control/src/OnlineCorrectionExceptionProcessor.cpp']
metadata={'metrics':metrics,'input_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
 'scope':'No input smoothing, fit, outlier removal or production changes. Phase-boundary differences excluded from jitter statistics.',
 'decomposition':'2D: Kprev*dC + (1-Kprev)*dS + dK*(Cnow-Snow). 3D: change both pixels at previous pose, then change pose at new pixels. Path-dependent accounting, not causal intervention.',
 'truth_limit':'No independent seam ground truth. Saved images end before largest events. Local calibration reproduces all logged fixed-base projections within 2e-6 mm.'}
(OUT/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')

# Read-only visualization of sampled image ROIs. Crosses represent recorded points, not new detections.
chosen=d[d.image_request_status.eq('queued')].iloc[[0,17,25,26,29]]
canvas=Image.new('RGB',(1100,5*270),'#f3f5f8');draw=ImageDraw.Draw(canvas)
font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',16) if Path('/System/Library/Fonts/Supplemental/Arial.ttf').exists() else ImageFont.load_default()
for j,(_,row) in enumerate(chosen.iterrows()):
    box=(int(row.enhancement_roi_x)-70,int(row.enhancement_roi_y)-40,int(row.enhancement_roi_x)+320,int(row.enhancement_roi_y)+240)
    for col,key in enumerate(('source_image','enhanced_image')):
        im=Image.open(SESSION/row[key]).convert('RGB');im=im.crop(box);pen=ImageDraw.Draw(im)
        for x,y,color in [(row.sim_x_px,row.sim_y_px,'#ff6262'),(row.before_fusion_x_px,row.before_fusion_y_px,'#65aaff'),(row.final_x_px,row.final_y_px,'#36e690')]:
            x-=box[0];y-=box[1];pen.line((x-7,y,x+7,y),fill=color,width=2);pen.line((x,y-7,x,y+7),fill=color,width=2)
        im.thumbnail((535,230));canvas.paste(im,(col*550+8,j*270+32))
        draw.text((col*550+8,j*270+8),f"Frame {int(row.frame_id)} | {'Source' if col==0 else 'Enhanced'} | {row.time_cn}",fill='#12213a',font=font)
canvas.save(OUT/'sampled_images.png')

js=ROOT/'output/analysis/20261019_110509_938_station_base/interactive_3d/plotly-2.35.2.min.js'
shutil.copy2(js,OUT/js.name)
# Compact data for an offline, locally reproducible interactive report.
fields=['frame_id','time_cn','t_s','phase','sim_x_px','sim_y_px','before_fusion_x_px','before_fusion_y_px','final_x_px','final_y_px','normalized_peak','kcf_weight_y','response_peak','template_reinitialized','station_base_z_mm','dkcf_x','dkcf_y','dfinal_x','dfinal_y','dfusion_x','dfusion_y','raw_term_x','prior_term_x','weight_term_x','pixel_dz_mm','pose_dz_mm','dz_mm','online_adjacent']
data=json.loads(d[fields].to_json(orient='records'))
trajectories=[{'name':'TCP 实际轨迹','color':'#2677bc','points':rq.tolist()},
 {'name':'最终检测投影（含融合）','color':'#e08a26','points':vq.tolist()},
 {'name':'本轮起始校正后的示教参考','color':'#248c63','points':corrected.tolist()},
 {'name':'原始示教参考（本机文件）','color':'#929aa3','points':original.tolist()}]
event_ids=[9034,9550,4961,5939,10878,5950]
buttons=''.join(f'<button class="event" data-frame="{n}">帧 {n}</button>' for n in event_ids)
keyrows=[]
for n in event_ids:
    q=d[d.frame_id.eq(n)].iloc[0]
    keyrows.append(f'<tr><td>{n}<br>{q.time_cn}</td><td>{q.dkcf_x:+.2f} / {q.dfinal_x:+.2f}</td><td>{q.dkcf_y:+.2f} / {q.dfinal_y:+.2f}</td><td>{q.dz_mm:+.3f}</td><td>{q.pixel_dz_mm:+.3f} / {q.pose_dz_mm:+.3f}</td><td>{int(q.template_reinitialized)}</td></tr>')
body='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>在线纠偏逐帧诊断｜2026-10-20</title><script src="plotly-2.35.2.min.js"></script>
<style>body{font:16px/1.7 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#f2f5f9;color:#1b2b40;margin:0}main{max-width:1200px;margin:auto;padding:30px 24px 70px}h1{font-size:30px;margin:5px 0}h2{font-size:23px}h3{font-size:18px}.muted,small{color:#52657c}section{background:white;border-radius:12px;padding:24px;margin:20px 0;border:1px solid #e0e6ef}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.card{padding:15px;background:#edf4fc;border-radius:8px}.card b{font-size:27px;display:block;color:#164e86}.warn{background:#fff4dd;padding:14px;border-radius:8px}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:10px 8px;text-align:left;border-bottom:1px solid #e5eaf1}button,input{font:inherit;padding:6px 13px;border:1px solid #c3cfdf;border-radius:6px;background:white;margin:4px;cursor:pointer}button.active{background:#164e86;color:white}a{color:#1461a3}.chart{height:760px}img{max-width:100%}@media(max-width:700px){.cards{grid-template-columns:1fr 1fr}.chart{height:800px}section{padding:12px}main{padding:14px}table{font-size:12px}}</style>
<main><small>只读分析 · 原始点保留 · 时间 Asia/Shanghai · 运行已正常完成</small><h1>绿色十字跳动：三种机制已有证据</h1>
<p class="muted">会话：20261020_155636_981_增加有过程记录的测试<br>从融合前二维点追到三维投影、ICP 批次和跟随输出；不将示教参考当作焊缝真值。</p>
<div class="cards"><div class="card"><b>2,124</b>完整有效诊断帧</div><div class="card"><b>15.04 px</b>在线相邻帧最大纵向跳变</div><div class="card"><b>3.781 mm</b>对应最大固定基座 |ΔZ|</div><div class="card"><b>30 对</b>PNG 全部校验通过 · 23.88 MiB</div></div>
<section><h2>结论与处理顺序</h2><ol><li><b>纵向大跳主要先出现在融合前 KCF 点。</b>在线阶段 1,748 个相邻帧对中，15 对 |Δv| &gt; 5 px；其中 14 对不是模板重建帧。帧 9034 的仿真 v 不动，KCF v 已跳 +15.46 px，最终 +15.04 px（同帧 Δu = +25.22 px），三维 ΔZ = −3.781 mm。应优先回放真实图像 → Gabor/Otsu → 相关响应，而非先平滑三维。</li><li><b>横向存在可直接复核的融合权重拉动。</b>帧 9550 的 KCF u 只变 −0.64 px、仿真 u 变 −1 px；权重从 0.9759 降至 0.7917，最终 u 反向跳 +15.61 px。当前两点相距约 88 px，使权重突变产生约 +16.26 px 的独立记账项。124 个 |Δu| &gt; 5 px 事件中，16 个权重变化项大于跟踪变化项。</li><li><b>模板重建与横向跳动明显伴随。</b>在线相邻帧中 34 次重建有 17 次 |Δu| &gt; 5 px（50%）；其他帧为 107/1714（6.24%）。本轮重建间隔严格为 51 个处理帧。重建时模型重训、滤波重新初始化都发生，单靠标记不能再拆分两者贡献。</li></ol><p>上述 5 px 是本分析筛选事件的阈值，不是产品精度或判废标准。相邻帧变化包含真实运动；未把它们全部命名为检测噪声。</p></section>
<section><h2>全程观察</h2><p>只显示在线阶段内部的相邻差分。15:57:27 前后由初始扫描进入在线跟随的约 190 px 位置变化已排除，不计入抖动。</p><div id="overview" class="chart"></div></section>
<section><h2>逐事件追踪</h2><p>切换事件或输入帧号，图表显示该帧附近的原始记录。竖线标出选中帧；虚线标出窗口内的重建帧。</p><div id="buttons">__BUTTONS__</div><label>帧号 <input id="frameInput" type="number" value="9034" style="width:110px"></label><button id="go">定位</button><p id="eventText" class="warn"></p><div id="eventPlot" class="chart"></div><h3>关键帧证据表</h3><table><thead><tr><th>帧 / 相机时间</th><th>Δu KCF / 最终 px</th><th>Δv KCF / 最终 px</th><th>ΔZ mm</th><th>像素 / 位姿项 mm</th><th>重建</th></tr></thead><tbody>__ROWS__</tbody></table><p class="muted">“像素项”先固定上一帧位姿、改变当前 u/v，再将位姿更新到本帧；两项之和复现实际 ΔZ。像素项包含 u/v 同时变化；以帧 9034 为例，先改 u 再改 v 的分项约为 −0.851 / −2.931 mm，位姿项 +0.001 mm。它是有顺序的数值拆账，不是物理干预实验。当前本机标定对全部 2,124 点的三维复算最大差为 8.52×10⁻⁷ mm。</p></section>
<section><h2>传到下游之后发生了什么</h2><p>共 319 个 ICP 批次，307 个应用、12 个回退原始点。帧 9034 可按坐标精确关联到第 193 批，该批 RMS 约 2.357 mm，最终回退原始点；这说明 ICP 没有保证滤除异常。所有 raw_pending 均通过本轮记录的坐标逐项匹配回源帧，见 CSV。</p><p>另有应用批次对原始批折线的最大改动达到 1.685 mm。该距离仅比较“释放目标”与“同批原始点折线”，不表示真实焊缝误差。首目标点可能是在传感器前视位置，不能把它与 TCP 的欧氏距离直接作为跟踪误差。</p><p>跟随计算记录的单次 |dx/dy/dz| 上限分别为 7/6/4 μm；这与检测点毫米级跳变是不同量。不能从检测 ΔZ 推断机器人 TCP 同幅跳动，也不能用正常完成证明轨迹精度达标。</p><h3>同一材料坐标中的原始轨迹</h3><p>统一到变位机末节系，单位 mm。检测点使用记录中的 positioner_tcp 加已知 TCP 偏置 (0,0,231 mm)；TCP 使用每条状态自己的外轴角转换。参考是本轮日志内的起始校正路径和本机原始示教文件，未用检测结果事后拟合整圈。可点击图例独立切换曲线。</p><div><button data-view="3d">三维</button><button data-view="xy">XY</button><button data-view="xz">XZ</button><button data-view="yz">YZ</button><button data-z="1">Z 1×</button><button data-z="8">Z 8×</button><span id="zlabel" class="muted">Z 显示倍率 1×，坐标值不变</span></div><div id="space" style="height:630px"></div></section>
<section><h2>图像证据与覆盖缺口</h2><p>红色：仿真先验；蓝色：融合前 KCF；绿色：最终点。只对原图与增强图做裁剪显示和记录点叠加，未重新检测或修饰条纹。</p><img src="sampled_images.png" alt="五组采样图像及同帧记录点"><p class="warn">30 对图中，25 对用于初始扫描，在线阶段只有 5 对，最晚到 15:57:34.220。主要异常发生在 15:58:12、15:59:53 和 16:00:54 附近，均没有同期图像。因此目前能区分跟踪、融合、重建，不能最终区分“源图细节变化、Gabor/Otsu 变化或相关峰误选”。采样图能看到反射纹理被二值化为竞争性亮结构，但不能用前段图像证明晚段根因。</p></section>
<section><h2>接下来怎样验证</h2><ol><li>先处理可重现的融合权重不连续：用本轮 KCF/先验输入做离线对照，比较连续权重与现策略的事件幅度及偏差，不直接上真实控制。</li><li>单独对照模板重建：固定同一输入，比较完整重置与保留滤波状态；不要同时更换预处理和融合，避免无法归因。</li><li>图像采样改为覆盖在线阶段，并对二维大跳/权重跨阈值/模板重建保留前后短窗口。数值仍逐帧，图像总预算继续受限；本分析未修改采样功能或任何生产代码。</li><li>在取得晚段图像后再评估相关峰质量门控、Gabor 区域和候选焊缝约束；只在三维末端加平滑不能修复已经发生的二维误选。</li></ol><p>时间方面，相机时间减状态接收时间范围 −176～169 ms；Vision 开始相对相机的中位延迟 101 ms，95 分位 185 ms；结果写入相对 Vision 结束的 95 分位延迟 281 ms。这些是主机时间指标，不能作为硬件同步精度。最大异常帧的固定姿态拆账已显示位姿项极小，但时序与控制积压仍值得单独验证。</p></section>
<section><h2>复核与文件</h2><p>所有输入只读，未平滑、删异常、重拟合或修改生产代码。二维加权关系和三维投影均逐帧复算校验。输入文件哈希和统计口径保存在 metadata.json。</p><p><a href="jump_events.csv">筛选事件 CSV</a> · <a href="diagnostics_enriched.csv">全部逐帧计算 CSV</a> · <a href="icp_raw_to_frame.csv">ICP 原始批与源帧对应</a> · <a href="icp_batch_effects.csv">ICP 批次影响</a> · <a href="metadata.json">来源与复核元信息</a> · <a href="build_report.py">生成脚本</a></p></section>
</main><script>
const D=__DATA__,T=__TRAJECTORIES__;const base={font:{family:'-apple-system,Segoe UI,sans-serif',size:12,color:'#243b55'},paper_bgcolor:'white',plot_bgcolor:'#f8fafc',margin:{l:65,r:25,t:30,b:55},hovermode:'x unified',legend:{orientation:'h',y:1.07},showlegend:true};
const color={kcf:'#2478bd',prior:'#dc5964',final:'#23956a'};
const A=D.filter(x=>x.online_adjacent);const ov=[];
function line(name,x,y,axis,col,dash){return {name,x,y,type:'scatter',mode:'lines',xaxis:axis===1?'x':'x'+axis,yaxis:axis===1?'y':'y'+axis,line:{color:col,width:1.5,dash:dash||'solid'}}}
['x','y'].forEach((a,i)=>{ov.push(line('Δ'+(a==='x'?'u':'v')+' 融合前',A.map(r=>r.t_s),A.map(r=>r['dkcf_'+a]),i+1,color.kcf));ov.push(line('Δ'+(a==='x'?'u':'v')+' 最终',A.map(r=>r.t_s),A.map(r=>r['dfinal_'+a]),i+1,color.final));});
ov.push(line('ΔZ 实际',A.map(r=>r.t_s),A.map(r=>r.dz_mm),3,'#694bb1'));ov.push(line('ΔZ 像素项',A.map(r=>r.t_s),A.map(r=>r.pixel_dz_mm),3,'#d38a22','dot'));
const ol={...base,grid:{rows:3,columns:1,pattern:'independent',roworder:'top to bottom'},yaxis:{title:'Δu / px'},yaxis2:{title:'Δv / px'},yaxis3:{title:'ΔZ / mm'},xaxis:{matches:'x3'},xaxis2:{matches:'x3'},xaxis3:{title:'距首次检测 / s'}};Plotly.newPlot('overview',ov,ol,{responsive:true,displaylogo:false});
function selectFrame(n){let j=D.reduce((best,r,i)=>Math.abs(r.frame_id-n)<Math.abs(D[best].frame_id-n)?i:best,0);let q=D[j],rows=D.slice(Math.max(0,j-10),Math.min(D.length,j+11));document.getElementById('frameInput').value=q.frame_id;document.querySelectorAll('.event').forEach(b=>b.classList.toggle('active',+b.dataset.frame===q.frame_id));const fmt=(v,n=3)=>v===null?'—':v.toFixed(n);document.getElementById('eventText').textContent=`帧 ${q.frame_id} · ${q.time_cn} · ${q.phase}｜Δu=${fmt(q.dfinal_x,2)} px，Δv=${fmt(q.dfinal_y,2)} px；ΔZ=${fmt(q.dz_mm)} mm = 像素项 ${fmt(q.pixel_dz_mm)} + 位姿项 ${fmt(q.pose_dz_mm)}；重建=${q.template_reinitialized}`;
const tr=[];['x','y'].forEach((a,i)=>{[['before_fusion','KCF 融合前',color.kcf],['sim','仿真先验',color.prior],['final','最终点',color.final]].forEach(([key,name,c])=>{const v=line(name+(i?' v':' u'),rows.map(r=>r.frame_id),rows.map(r=>r[key+'_'+a+'_px']),i+1,c);v.mode='lines+markers';v.marker={size:4};tr.push(v);});});
tr.push(line('KCF 权重',rows.map(r=>r.frame_id),rows.map(r=>r.kcf_weight_y),3,'#7253ad'));tr.push(line('归一化峰值',rows.map(r=>r.frame_id),rows.map(r=>r.normalized_peak),3,'#dc8c1d'));
tr.push(line('ΔZ 实际',rows.map(r=>r.frame_id),rows.map(r=>r.dz_mm),4,'#6444a3'));tr.push(line('ΔZ 像素项',rows.map(r=>r.frame_id),rows.map(r=>r.pixel_dz_mm),4,'#d38a22','dot'));tr.push(line('ΔZ 位姿项',rows.map(r=>r.frame_id),rows.map(r=>r.pose_dz_mm),4,'#62798a'));
const shapes=[{type:'line',xref:'x',yref:'paper',x0:q.frame_id,x1:q.frame_id,y0:0,y1:1,line:{color:'#263d57',width:1}}];rows.filter(r=>r.template_reinitialized).forEach(r=>shapes.push({type:'line',xref:'x',yref:'paper',x0:r.frame_id,x1:r.frame_id,y0:0,y1:1,line:{color:'#b44a68',width:1,dash:'dot'}}));
Plotly.react('eventPlot',tr,{...base,grid:{rows:4,columns:1,pattern:'independent'},yaxis:{title:'u / px'},yaxis2:{title:'v / px'},yaxis3:{title:'权重 / 归一化峰'},yaxis4:{title:'ΔZ / mm'},xaxis:{matches:'x4'},xaxis2:{matches:'x4'},xaxis3:{matches:'x4'},xaxis4:{title:'相机帧号（并非连续处理序号）'},shapes},{responsive:true,displaylogo:false});}
document.querySelectorAll('.event').forEach(b=>b.onclick=()=>selectFrame(+b.dataset.frame));document.getElementById('go').onclick=()=>selectFrame(+document.getElementById('frameInput').value);selectFrame(9034);
const space=T.map((r,i)=>({type:'scatter3d',name:r.name,x:r.points.map(p=>p[0]),y:r.points.map(p=>p[1]),z:r.points.map(p=>p[2]),mode:'lines',line:{color:r.color,width:i===1?3:4,dash:i===3?'dash':'solid'},hovertemplate:'x=%{x:.3f} mm<br>y=%{y:.3f} mm<br>z=%{z:.3f} mm<extra>%{fullData.name}</extra>'}));
const ranges=[0,1,2].map(i=>{const a=T.flatMap(r=>r.points.map(p=>p[i]));return Math.max(...a)-Math.min(...a)});const ar={x:1,y:ranges[1]/ranges[0],z:ranges[2]/ranges[0]};Plotly.newPlot('space',space,{...base,hovermode:'closest',margin:{l:0,r:0,t:20,b:0},scene:{xaxis:{title:'X / mm'},yaxis:{title:'Y / mm'},zaxis:{title:'Z / mm'},aspectmode:'manual',aspectratio:ar,camera:{eye:{x:1.4,y:1.4,z:1}}}},{responsive:true,displaylogo:false});
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{const views={'3d':{x:1.4,y:1.4,z:1},xy:{x:0,y:0,z:2.5},xz:{x:0,y:-2.5,z:0},yz:{x:2.5,y:0,z:0}};const up=b.dataset.view==='xy'?{x:0,y:1,z:0}:{x:0,y:0,z:1};Plotly.relayout('space',{'scene.camera.eye':views[b.dataset.view],'scene.camera.up':up});});document.querySelectorAll('[data-z]').forEach(b=>b.onclick=()=>{let k=+b.dataset.z;Plotly.relayout('space',{'scene.aspectratio':{x:ar.x,y:ar.y,z:ar.z*k}});document.getElementById('zlabel').textContent=`Z 显示倍率 ${k}×，坐标值不变`;});
</script></html>'''
body=body.replace('__BUTTONS__',buttons).replace('__ROWS__',''.join(keyrows)).replace('__DATA__',json.dumps(data,ensure_ascii=False,separators=(',',':'))).replace('__TRAJECTORIES__',json.dumps(trajectories,ensure_ascii=False,separators=(',',':')))
# Use the actual log-derived image coverage time to prevent prose drift.
body=body.replace('15:57:34.220',metrics['image_last_camera_time'])
(OUT/'report.html').write_text(body,encoding='utf-8')
print(json.dumps(metrics,ensure_ascii=False,indent=2))
print('Report:',OUT/'report.html')
