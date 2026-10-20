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
SESSION = ROOT / 'output/online_correction/20261020_162854_226'

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
assert len(d)>0 and not d.frame_id.duplicated().any()
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
assert len(adj)==len(on)-1
xjump=adj[adj.dfinal_x.abs()>5];yjump=adj[adj.dfinal_y.abs()>5]
assert len(xjump)>0
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

# Compare identical definitions; neither run has independent seam ground truth.
previous=pd.read_csv(ROOT/'output/analysis/20261020_155636_981_diagnostics/diagnostics_enriched.csv')
comparison=[]
for label,frame in [('15:56 五P点记录',previous),('16:28 P001+四固定点（用户说明）',d)]:
    a=frame[frame.online_adjacent];o=frame[frame.phase.eq('online_running')]
    re=a.template_reinitialized.eq(1)
    row={'session':label,'online_frames':len(o),'pairs':len(a),
         'duration_s':(o.camera_timestamp_ms.iloc[-1]-o.camera_timestamp_ms.iloc[0])/1000}
    for axis in 'xy':
        jump=a[f'dfinal_{axis}'].abs()>5
        row[f'{axis}_jump_count']=int(jump.sum());row[f'{axis}_jump_percent']=float(jump.mean()*100)
        row[f'{axis}_jump_reinit_count']=int((jump&re).sum())
        row[f'{axis}_jump_weight_dominant']=int((jump&a[f'dominant_{axis}'].eq('weight_change')).sum())
    row['reinit_count']=int(re.sum())
    for col in ['dfinal_x','dfinal_y','dz_mm','pixel_dz_mm','pose_dz_mm']:
        for qv,tag in [(.95,'p95'),(1,'max')]:row[f'{col}_{tag}']=float(a[col].abs().quantile(qv))
    comparison.append(row)
pd.DataFrame(comparison).to_csv(OUT/'comparison.csv',index=False,encoding='utf-8-sig')

def stamp(ms):return pd.to_datetime(ms,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3]
phase_rows=[]
for phase,g in robot.groupby('phase',sort=False):
    phase_rows.append({'phase':phase,'start':stamp(g.timestamp_ms.iloc[0]),'end':stamp(g.timestamp_ms.iloc[-1]),'count':len(g)})
pd.DataFrame(phase_rows).to_csv(OUT/'robot_phase_timeline.csv',index=False,encoding='utf-8-sig')
wait=robot[robot.phase.eq('waiting_robot_program_stop')].copy()
wait.to_csv(OUT/'exit_robot_states.csv',index=False,encoding='utf-8-sig')
on_robot=robot[robot.phase.eq('online_running')]
summary=dict(zip(load('session_summary.txt')['key'],load('session_summary.txt')['value']))
metadata={'session':SESSION.name,'comparison':comparison,'summary':summary,
          'projection_max_error_mm':float(residual.max()),
          'image_pairs':len(manifest),'image_last_camera_time':d[d.image_request_status.eq('queued')].time_cn.iloc[-1],
          'icp_batches':len(b),'icp_applied':int(b.applied.sum()),
          'raw_pending_count':len(raw),'matched_raw_pending_count':int(raw.source_frame_id.notna().sum()),
          'max_released_to_raw_polyline_mm':float(bs.max_to_raw_polyline_mm.max()),
          'max_follow_command_um':follow[['dx_um','dy_um','dz_um']].abs().max().to_dict(),
          'input_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in list(SESSION.rglob('*'))+[sensor,rb] if p.is_file()},
          'limits':['No independent seam ground truth. Adjacent differences include motion and variable sampling interval.',
                    'Current local calibration reproduces logged station projections; not an independent physical calibration check.',
                    'No controller-installed JBI in archive; five O messages do not prove all five points were executed.',
                    'Pixel/pose decomposition is ordered endpoint accounting, not a causal intervention.']}
(OUT/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
ids=[10317,10345,14589,14712,14717,10494]
event=d[d.frame_id.isin(ids)]
cols=['frame_id','time_cn','dkcf_x','dfinal_x','dkcf_y','dfinal_y','dz_mm','pixel_dz_mm','pose_dz_mm','weight_term_x','template_reinitialized']
event[cols].to_csv(OUT/'key_events.csv',index=False,encoding='utf-8-sig')
shutil.copy2(ROOT/'output/analysis/20261020_155636_981_diagnostics/plotly-2.35.2.min.js',OUT/'plotly-2.35.2.min.js')
# Reuse previously verified chart interactions, with this run's own records and event IDs.
old=(ROOT/'output/analysis/20261020_155636_981_diagnostics/build_report.py').read_text()
chartjs=old.split('const D=__DATA__')[1].split('</script></html>')[0]
chartjs='const D=__DATA__'+chartjs
chartjs=chartjs.replace('selectFrame(9034);','selectFrame(10317);')
fields=['frame_id','time_cn','t_s','phase','sim_x_px','sim_y_px','before_fusion_x_px','before_fusion_y_px','final_x_px','final_y_px','normalized_peak','kcf_weight_y','response_peak','template_reinitialized','station_base_z_mm','dkcf_x','dkcf_y','dfinal_x','dfinal_y','dfusion_x','dfusion_y','raw_term_x','prior_term_x','weight_term_x','pixel_dz_mm','pose_dz_mm','dz_mm','online_adjacent']
trajectories=[{'name':'实际TCP（在线段）','color':'#2677bc','points':rq.tolist()},
              {'name':'最终检测投影','color':'#e08a26','points':vq.tolist()},
              {'name':'起始校正参考路径','color':'#248c63','points':corrected.tolist()}]
chartjs=chartjs.replace('__DATA__',d[fields].to_json(orient='records')).replace('__TRAJECTORIES__',json.dumps(trajectories,ensure_ascii=False))
table=pd.DataFrame(comparison).set_index('session').T.to_html(float_format=lambda v:f'{v:.4f}')
buttons=''.join(f'<button class="event" data-frame="{n}">帧{n}</button>' for n in ids)
body='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>P001＋四固定点在线纠偏复核</title><script src="plotly-2.35.2.min.js"></script>
<style>body{margin:auto;max-width:1250px;padding:28px;background:#f4f6fa;color:#24364c;font:16px/1.7 -apple-system,sans-serif}section{background:white;padding:24px;margin:20px 0;border-radius:12px}h1{font-size:30px}h2{font-size:23px}table{border-collapse:collapse;font-size:13px;width:100%}td,th{padding:7px;border-bottom:1px solid #dde4ee;text-align:left}button,input{padding:8px;margin:4px;border:1px solid #bbc9da;border-radius:5px;background:white}.chart{height:780px}.warn{background:#fff1d5;padding:16px}a{color:#1461a3}</style>
<h1>P001＋四固定点：检测变化减小，但退出运动触发超时</h1>
<p>会话20261020_162854_226 · 与15:56会话同口径对照 · 时间Asia/Shanghai</p>
<section><h2>核心判断</h2><p><b>本轮不是完整正常结束的试验。</b>16:41:07.799进入等待主程序停止，之后纠偏/检测标志已清零，机器人仍运动；16:41:15.247起停留在行21，至16:43:07.732触发120秒超时。等待期机器人TCP累计移动约762.87 mm。应核对实际控制柜行21及退出段目标、速度和外轴运动路径，不能直接认定结束P命令未收到。</p>
<p>在线段新旧均约一圈：S2累计−359.63°与−359.96°，平均角速度0.523°/s与1.585°/s。新运行约为旧的三分之一角速度，没有多圈证据。退出等待期S2反向连续走约+321.97°，是需要核查的长回转。仓库20260914CFKONTEST0.JBR第63–71行与日志行号吻合：14–18为P001加四固定点，21为MOVJ C00011及EC00011（外轴VJ=0.78）；这只是现场程序候选，尚未确认安装版本。</p><p><b>检测相邻帧变化有所收敛，原有机制仍然存在。</b>在线6179帧、6178个相邻对，|ΔZ| P95=0.338 mm、最大1.420 mm，上一轮为0.412/3.781 mm；|Δu|&gt;5 px比例7.09%→3.69%，|Δv|&gt;5 px比例0.86%→0.26%。但在线时长227→687秒，运行条件与采样间隔不同，不能由此证明固定示教方案因果性改善精度。</p>
<p><b>融合权重仍可使输出反向跳动。</b>帧14589融合前u变−11.29 px，最终u却变+9.83 px；权重变化项约+20.84 px。121次在线重建中24次横向跳变&gt;5 px（19.83%），非重建为204/6057（3.37%），仍有关联。帧10317的ΔZ=−1.420 mm，位姿项约0，变化发生在像素输入侧。</p>
<p class="warn">用户说明现场采用P001＋四固定点。压缩包只有日志与图像，没有控制柜JBI。日志仍记载5条O写点命令；这证明PC发出5点，不证明JBI执行了5个动态P点，更不能以仓库JBI替代现场程序。</p></section>
<section><h2>全程与逐帧证据</h2><p>5 px为事件筛选阈值，不是产品精度标准；相邻差分包含真实运动。阶段切换已排除。下图横轴从本会话首次检测计时。</p><div id="overview" class="chart"></div>
__BUTTONS__<input id="frameInput" type="number" value="10317"><button id="go">定位帧</button><p id="eventText" class="warn"></p><div id="eventPlot" class="chart"></div>__EVENTS__</section>
<section><h2>材料坐标中的原始轨迹</h2><p>单位mm。检测使用记录的变位机TCP坐标加(0,0,231 mm)偏置；实际TCP逐条按自身外轴角变换。参考来自本会话initial_correction，未重新拟合检测结果。这里的检测点在传感器前视处，不能将它与同时刻TCP的距离视为跟踪误差。</p>
<button data-view="3d">3D</button><button data-view="xy">XY</button><button data-view="xz">XZ</button><button data-view="yz">YZ</button><button data-z="1">Z 1×</button><button data-z="8">Z 8×</button><span id="zlabel">Z显示倍率1×</span><div id="space" style="height:650px"></div></section>
<section><h2>退出段原始状态</h2><p>下图保留实际机器人坐标与外轴；行21不等于程序卡死，必须结合持续运动判断。</p><div id="exitplot" style="height:650px"></div></section>
<section><h2>下游与收尾代码核对</h2><p>413批ICP中355批应用、58批回退原始点；应用/释放点对同批原始折线的最大距离1.944 mm，不是焊缝误差。6179条raw_pending全部关联到源帧。跟随记录单次dx/dy/dz绝对值上限7/6/4 μm，不能把检测点毫米级变化等同于TCP同幅运动。</p><p>Control/src/OnlineCorrectionController.cpp:443–472在最终detflag清零后等待sStart=0；UI/src/RuntimeWorkerAdapters.cpp:2751–2778设置固定120秒超时，持续运动不续期。Control/src/OnlineCorrectionController.cpp:560–567进入Error并停止tracking，没有在该处发机器人hold/stop，日志结束不代表机器人停止。</p></section><section><h2>对照统计与限制</h2>__TABLE__
<p>全部6410帧均记录有效，跳变保护均未触发。图像30对，其中初始扫描15对、发点阶段1对、在线14对；最晚16:30:03.681，未覆盖16:32:53和16:35:14等关键事件。因此不能把融合前异常进一步归因于原图、增强或相关峰误选。</p>
<p>三维投影按本机标定逐帧复算，最大残差见metadata.json；这核对计算一致性，不代替物理标定精度验收。没有独立焊缝真值，不能据此判定机器人实际跟踪精度。</p>
<p>建议先核对并导出本轮控制柜JBI退出段，再分别做相同速度/同段路径的示教方案对照，以及连续融合权重与重建状态保留的离线对照。图像预算需覆盖晚段异常。当前未修改生产算法或JBI。</p>
<p><a href="comparison.csv">对照统计</a> · <a href="key_events.csv">关键事件</a> · <a href="jump_events.csv">全部筛选事件</a> · <a href="diagnostics_enriched.csv">完整逐帧计算</a> · <a href="exit_robot_states.csv">退出段状态</a> · <a href="icp_batch_effects.csv">ICP批次影响</a> · <a href="metadata.json">输入哈希及限制</a> · <a href="analyze.py">复现脚本</a></p></section>
<script>__JS__</script></html>'''
exitdata=json.loads(wait[['timestamp_ms','x_mm','y_mm','z_mm','s2_deg','job_line','s_start']].to_json(orient='records'))
exitjs='const E='+json.dumps(exitdata)+';'+'''const ex=[];['x_mm','y_mm','z_mm'].forEach((k,i)=>ex.push(line(k,E.map(r=>(r.timestamp_ms-E[0].timestamp_ms)/1000),E.map(r=>r[k]),1,['#2478bd','#dc5964','#23956a'][i])));['s2_deg','job_line','s_start'].forEach((k,i)=>ex.push(line(k,E.map(r=>(r.timestamp_ms-E[0].timestamp_ms)/1000),E.map(r=>r[k]),i+2,['#7253ad','#dc8c1d','#62798a'][i])));Plotly.newPlot('exitplot',ex,{...base,grid:{rows:4,columns:1,pattern:'independent'},yaxis:{title:'机器人XYZ/mm'},yaxis2:{title:'S2/deg'},yaxis3:{title:'JBI行'},yaxis4:{title:'sStart'},xaxis:{matches:'x4'},xaxis2:{matches:'x4'},xaxis3:{matches:'x4'},xaxis4:{title:'进入等待后/s'}},{responsive:true,displaylogo:false});'''
body=body.replace('__BUTTONS__',buttons).replace('__EVENTS__',event[cols].to_html(index=False,float_format=lambda v:f'{v:.3f}')).replace('__TABLE__',table).replace('__JS__',chartjs+exitjs)
(OUT/'report.html').write_text(body,encoding='utf-8')
print(json.dumps({k:v for k,v in metadata.items() if k!='input_hashes'},ensure_ascii=False,indent=2))

