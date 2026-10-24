"""Revise this session to analytical fitted-circle errors; retain previous artifacts."""
from pathlib import Path
import ast, hashlib, html, importlib.util, json, shutil, sys
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
from PIL import Image,ImageDraw,ImageFont
OUT=Path(__file__).resolve().parent
BACKUP=OUT/'polyline_version_backup';BACKUP.mkdir(exist_ok=True)
for name in ['results.json','metrics.csv','report.html','index.html','preview.png','viewer_data.json','verification.json','viewer_verification.json']:
    src=OUT/name;dst=BACKUP/name
    if src.exists() and not dst.exists():shutil.copy2(src,dst)
r=json.loads((BACKUP/'results.json').read_text(encoding='utf-8'))
def dump(name,value):
    (OUT/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a.item()),encoding='utf-8')
def load(name):return pd.read_csv(OUT/name)
spec=importlib.util.spec_from_file_location('geometry',OUT.parents[2]/'output/analysis/20261020_evening_comparison/assets/geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
R=np.array(r['alignment']['rotation']);t=np.array(r['alignment']['translation'])
wc=r['original_circle'];kc=r['raw_kcf_circle']
center=R@np.array(wc['center'])+t;normal=R@np.array(wc['normal']);normal/=np.linalg.norm(normal)
radius=wc['radius_mm'];reference=dict(center=center,normal=normal,radius_mm=radius)
def distance(q,c,n,rad):
    c=np.asarray(c);n=np.asarray(n);n=n/np.linalg.norm(n)
    delta=q-c;h=delta@n;planar=delta-h[:,None]*n;rho=np.linalg.norm(planar,axis=1)
    u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u)
    direction=np.divide(planar,rho[:,None],out=np.tile(u,(len(q),1)),where=rho[:,None]>1e-12)
    foot=c+rad*direction
    return np.hypot(rho-rad,h),foot,rho-rad,h
assert np.allclose(distance(np.array([[6,2,3],[8,2,3],[1,2,3],[6,2,6],[1,2,15.]]),[1,2,3],[0,0,1],5)[0],[0,2,5,3,13])
metrics={};series={};inputs={}
for name,file in [('TCP','tcp_material_frame.csv'),('raw_KCF','raw_kcf_material_frame.csv'),('final_vision','final_vision_material_frame.csv')]:
    frame=load(file);q=frame[['x_mm','y_mm','z_mm']].to_numpy();times=frame.timestamp_ms.to_numpy()
    error,foot,radial,height=distance(q,center,normal,radius)
    # Independent all-point checks against existing geometry and circle constraints.
    assert np.max(abs(error-g.circle_distance(q,reference)))<1e-10
    assert np.max(abs(np.linalg.norm(q-foot,axis=1)-error))<1e-10
    assert np.max(abs((foot-center)@normal))<1e-10
    assert np.max(abs(np.linalg.norm(foot-center,axis=1)-radius))<1e-10
    stats=g.metric(error);imax=int(error.argmax())
    stats.update(max_time=pd.to_datetime(times[imax],unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3],within_2mm_fraction=float(np.mean(error<=2)),within_3mm_fraction=float(np.mean(error<=3)),above_3mm_count=int(np.sum(error>3)))
    key=name+'_to_moved_teach_circle';metrics[key]=stats;series[name]=(times,error)
    frame['distance_mm']=error;frame['signed_radial_mm']=radial;frame['signed_plane_height_mm']=height
    for j,a in enumerate('xyz'):frame['foot_'+a+'_mm']=foot[:,j]
    frame.to_csv(OUT/(key+'.csv'),index=False,encoding='utf-8-sig')
    inputs[str(OUT/file)]=hashlib.sha256((OUT/file).read_bytes()).hexdigest()
    # Dense independent angular search on representative points: upper-bound check.
    n=normal;u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u);v=np.cross(n,u)
    angles=np.linspace(0,2*np.pi,65536,endpoint=False);samples=center+radius*(np.cos(angles)[:,None]*u+np.sin(angles)[:,None]*v)
    for i in np.unique(np.r_[np.linspace(0,len(q)-1,11,dtype=int),imax]):
        dense=np.min(np.linalg.norm(samples-q[i],axis=1))
        assert dense+1e-10>=error[i] and dense-error[i]<.007
# Re-evaluate previous holdout centers/normals using a continuous circle.
fold=load('raw_kcf_cross_validation.csv').fold.to_numpy();q=load('raw_kcf_material_frame.csv').iloc[:,1:4].to_numpy();cv=np.empty(len(q))
for a in r['cross_validation_folds']:
    mask=fold==a['held_out_fold'];cv[mask]=distance(q[mask],a['center'],a['normal'],radius)[0]
r['previous_polyline_metrics']=r['metrics'];r['metrics']=metrics
r['previous_polyline_diagnostics']={key:r.pop(key) for key in ['reference_alternatives','cross_validation_folds','cross_validation_reference_displacement','tcp_phase_metrics']}
r['reference_geometry']='Analytical continuous 3D fitted circle of original teaching, rigidly relocated; original fitted radius preserved'
r['moved_teach_circle']=reference
r['method']='Fit original teach circle; align its center and normal to angle-balanced raw KCF; keep original circle radius. Evaluate exact point-to-circle 3D distance, not chords or sampled vertices.'
r['acceptance_context']='User: simulator workpiece model comes from surveying; actual workpiece manufacturing variation is large; approximately 2–3 mm deviation is acceptable. Report fractions at both 2 and 3 mm without treating this as a strict all-points pass limit.'
r['cross_validation_pooled']=g.metric(cv)
r['limitations']=[a for a in r['limitations'] if 'finite polyline' not in a and 'yaw' not in a]+['Point-to-continuous-circle geometry does not evaluate along-seam timing or tool orientation. Circle geometry is invariant to axial yaw.']
dump('results.json',r);pd.DataFrame([dict(metric=k,**v) for k,v in metrics.items()]).to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
dump('moved_teach_circle.json',reference)
pd.DataFrame({'fold':fold,'distance_mm':cv}).to_csv(OUT/'raw_kcf_circle_cross_validation.csv',index=False,encoding='utf-8-sig')
# Default both teaching overlays to their fitted circles; retain raw teaching as options.
viewer=json.loads((BACKUP/'viewer_data.json').read_text(encoding='utf-8'))
for i in [2,3]:viewer['traces'][i]['visible']=True;viewer['traces'][i]['name']=viewer['traces'][i]['name'].replace('（辅助）','')
for i in [4,5]:viewer['traces'][i]['visible']=False
dump('viewer_data.json',viewer)
page=(BACKUP/'index.html').read_text(encoding='utf-8')
start=page.rfind('const traces = ')+len('const traces = ');end=page.index(';\nconst ranges',start)
page=page[:start]+json.dumps(viewer['traces'],ensure_ascii=False)+page[end:]
page=page.replace('<option value="actual" selected>','<option value="actual">').replace('<option value="fit">','<option value="fit" selected>')
start=page.index('<footer>');end=page.index('</footer>',start)
page=page[:start]+'<footer><a href="report.html">返回拟合圆分析报告</a> · 灰/绿默认显示原始/移动后示教拟合圆，原始点仍可切换。TCP与KCF保留全部有效观测点。误差按点到连续三维拟合圆解析计算；不使用圆的离散显示线段。移动后圆保留原始示教拟合半径，是KCF定位的估计参考。用户确认约2～3 mm误差可接受。Z倍率不改变坐标与统计。未使用鼠标，真实浏览器交互未验收。</footer>'+page[end+len('</footer>'):]
(OUT/'index.html').write_text(page,encoding='utf-8')
# Reuse the previous chart renderer, without executing its analysis pipeline.
im=Image.new('RGB',(1800,1250),'white');draw=ImageDraw.Draw(im)
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',24);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',19)
tree=ast.parse((OUT/'analyze.py').read_text(encoding='utf-8-sig'));fn=next(a for a in tree.body if isinstance(a,ast.FunctionDef) and a.name=='chart')
exec(compile(ast.Module(body=[fn],type_ignores=[]),'chart_renderer','exec'))
draw.text((50,15),'2026-10-21 15:23 复测：拟合圆评价',fill='#203345',font=font)
draw.text((50,55),'误差为点到连续三维圆的最短距离；原始示教拟合半径不变；约2～3 mm为用户可接受量级',fill='#526576',font=small)
colors=['#777777','#0072b2','#d55e00','#009e73'];labels=['原始示教拟合圆','TCP 实测','KCF 融合前','移动后示教拟合圆']
for i,(c,label) in enumerate(zip(colors,labels)):draw.text((50+i*420,95),label,fill=c,font=small)
traces=viewer['traces'];curves=[np.c_[traces[i]['x'],traces[i]['y'],traces[i]['z']] for i in [2,0,1,3]]
for box,(i,j,title) in zip([(125,225,850,625),(1025,225,1750,625)],[(0,1,'XY 投影（等比例）'),(0,2,'XZ 投影（等比例）')]):chart(box,[(a[:,i],a[:,j],c) for a,c in zip(curves,colors)],title,'XYZ'[i]+' (mm)','XYZ'[j]+' (mm)',True)
start=min(series['TCP'][0][0],series['raw_KCF'][0][0])
chart((125,825,850,1165),[((times-start)/1000,e,c) for (times,e),c in [(series['TCP'],colors[1]),(series['raw_KCF'],colors[2])]],'全部有效在线点误差','在线时间 (s)','到移动后示教拟合圆距离 (mm)')
chart((1025,825,1750,1165),[(np.sort(e),np.arange(1,len(e)+1)/len(e),c) for (_,e),c in [(series['TCP'],colors[1]),(series['raw_KCF'],colors[2])]],'误差累计分布','距离 (mm)','累计比例')
im.save(OUT/'preview.png')
def table():
    rows=''
    for key,label in [('TCP_to_moved_teach_circle','TCP'),('raw_KCF_to_moved_teach_circle','原始KCF'),('final_vision_to_moved_teach_circle','融合后视觉（补充）')]:
        a=metrics[key];rows+=f"<tr><td>{label}</td><td>{a['n']}</td><td>{a['mean_mm']:.4f}</td><td>{a['max_mm']:.4f}</td><td>{a['rmse_mm']:.4f}</td><td>{a['p95_mm']:.4f}</td><td>{a['within_2mm_fraction']:.2%}</td><td>{a['within_3mm_fraction']:.2%}</td><td>{a['max_time']}</td></tr>"
    return '<table><tr><th>轨迹</th><th>点数</th><th>平均/mm</th><th>最大/mm</th><th>RMSE/mm</th><th>P95/mm</th><th>≤2 mm</th><th>≤3 mm</th><th>最大值时刻</th></tr>'+rows+'</table>'
a=metrics['TCP_to_moved_teach_circle'];b=metrics['raw_KCF_to_moved_teach_circle']
report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>在线纠偏复测｜拟合圆评价</title><style>body{{max-width:1200px;margin:35px auto;padding:0 25px;font:16px/1.8 system-ui,"Microsoft YaHei";color:#203345}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{border-bottom:1px solid #dce5ec;padding:9px;text-align:left}}th{{background:#edf4fa}}.note{{padding:15px;background:#eaf5f0}}img{{max-width:100%}}a{{color:#0072b2}}pre{{background:#f5f7f9;padding:15px;white-space:pre-wrap}}</style>
<h1>在线纠偏复测：改用示教拟合圆评价</h1><p>会话 20261021_152328_050；2026-10-21 15:23–15:33。<a href="index.html">四轨迹交互图</a> · <a href="metrics.csv">统计表</a> · <a href="results.json">完整结果</a></p>
<p class="note"><b>按用户最新要求，主要误差全部改为点到移动后原始示教拟合圆的最短三维距离。</b>仿真工件模型由测绘得到，真实工件制造误差也较大，用户认可约2～3 mm偏差可接受。因此半径差仅作几何描述，不再作为需要优先排查的问题；不凭它判断局部配准或控制异常。</p>
<h2>结果与可接受范围</h2>{table()}
<p>TCP平均 {a['mean_mm']:.3f} mm、RMSE {a['rmse_mm']:.3f} mm；KCF平均 {b['mean_mm']:.3f} mm、RMSE {b['rmse_mm']:.3f} mm。结合用户给出的约2～3 mm可接受量级，<b>本轮整体表现可接受</b>，同时保留局部峰值事实：TCP有{a['above_3mm_count']}点、KCF有{b['above_3mm_count']}点超过3 mm。用户未将3 mm定义为严格的逐点最大误差限值，因此不据此宣称整段所有点均达标，也不将少量超出直接判为整轮失败。</p>
<h2>拟合圆和移动方式</h2><p>保留上一版的圆拟合方法与定位变换，确保本次只改变评价几何：原始示教经2°角度分箱代表点拟合三维圆；圆心/法线按融合前KCF整圈估计结果刚体移动，<b>半径仍为原始示教拟合半径 {radius:.6f} mm</b>，没有改用KCF自身半径。移动后圆心为 {html.escape(str(center.tolist()))} mm；单位法线为 {html.escape(str(normal.tolist()))}。<a href="moved_teach_circle.json">圆参数</a> · <a href="teach_to_estimated_current_transform.csv">刚体变换</a>。</p>
<p>四条主要显示曲线为原始示教拟合圆、TCP原始轨迹、KCF原始轨迹、移动后示教拟合圆；两组原始示教点仍可作为补充切换，默认显示拟合圆。TCP、KCF未拟合、平滑或删去异常点。圆不依赖绕自身法线的相位角，因此上一版原始非圆轨迹物理相位的不确定性不再影响本次圆距离。</p>
<h2>误差口径</h2><pre>c：移动后圆心；n：圆平面单位法线；r：原始示教拟合半径
h = (p-c)·n
ρ = ||(p-c)-h n||
e = sqrt((ρ-r)²+h²)
平均误差 = mean(e)；最大误差 = max(e)；RMSE = sqrt(mean(e²))</pre>
<p>这是到空间连续圆周的解析最短距离，包含径向与离面分量；不是仅径向差，也不是到圆盘、圆的显示采样点或显示折线的距离。统计仍为逐记录样本加权，沿用全部有效在线TCP {r['tcp_count']} 点、原始KCF {r['raw_kcf_count']} 点和同帧坐标变换；不评价沿缝时序滞后与工具姿态角。逐点CSV包含径向、离面分量及圆周垂足，便于复算。</p>
<h2>对上一版判断的修正</h2><p>原始示教拟合半径 {radius:.3f} mm，KCF拟合半径 {kc['radius_mm']:.3f} mm，差约 {abs(radius-kc['radius_mm']):.3f} mm。在测绘建模与真实工件制造误差背景下，这个差异不能直接归因于检测或纠偏故障；当前无须单独优先追查。评价重点改为整体误差水平、2/3 mm范围内的比例、局部峰值及其连续性。TCP与KCF接近表示几何一致性较好，不单凭接近程度判断绝对精度。</p>
<img src="preview.png" alt="拟合圆四轨迹和解析圆距离统计">
<h2>参考性质与验证</h2><p>移动后示教圆仍由KCF估计位置，是本轮比较参考，未新增独立测量真值。用前一版相同的交替扇区留出参考改按圆计算，KCF留出均值 {r['cross_validation_pooled']['mean_mm']:.4f} mm、RMSE {r['cross_validation_pooled']['rmse_mm']:.4f} mm；留出验证减少直接拟合复用，不能消除共享标定/检测偏差。</p>
<p>原始采集、标定回放与输入哈希沿用已验证结果。新增圆距离已与另一实现逐点交叉检查，并检查圆周垂足的平面/半径约束、已知答案合成点和稠密圆周独立搜索。静态图已检查；不使用鼠标，真实浏览器交互未验收。<a href="circle_verification.json">本次验证</a> · <a href="input_hashes.json">原始输入哈希</a> · <a href="circle_input_hashes.json">本次复算输入哈希</a>。</p>
<p>只更新派生分析产物，未修改业务源码和原始数据，未删除文件。旧折线口径成果保存在 <a href="polyline_version_backup/report.html">上一版报告快照</a>（旧页内相对附件路径需回到父目录查阅），不再作为本版主要结论。复现本版请运行同目录 <code>revise_circle.py</code>；旧 <code>analyze.py/build_report.py</code> 是原折线流程，不用于生成本版。</p></html>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
for p in [BACKUP/'results.json',BACKUP/'viewer_data.json',BACKUP/'index.html',OUT/'raw_kcf_cross_validation.csv',OUT/'analyze.py']:
    inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
dump('circle_input_hashes.json',inputs)
for p,h in json.loads((OUT/'input_hashes.json').read_text(encoding='utf-8')).items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
for key,a in metrics.items():
    e=pd.read_csv(OUT/(key+'.csv')).distance_mm.to_numpy()
    assert len(e)==a['n'] and abs(e.mean()-a['mean_mm'])<1e-12 and abs(np.sqrt(np.mean(e**2))-a['rmse_mm'])<1e-12
    assert abs(float(np.mean(e<=3))-a['within_3mm_fraction'])<1e-12
script=(OUT/'verify_viewer.cjs').read_text(encoding='utf-8')
script=script.replace("data[2].visible,false","data[2].visible,true").replace("data[3].visible,false","data[3].visible,true").replace("data[4].visible,true","data[4].visible,false").replace("data[5].visible,true","data[5].visible,false").replace("'viewer_verification.json'","'circle_viewer_verification.json'")
(OUT/'verify_circle_viewer.cjs').write_text(script,encoding='utf-8')
dump('circle_verification.json',dict(known_answer_points='passed',all_point_geometry_cross_check='passed',circle_foot_plane_and_radius_constraints='passed',dense_angular_search='passed',radius_preserved=True,reference_geometry='analytical continuous circle',original_data_and_code='unchanged',actual_browser_ui='not performed',mouse='not used'))
print(json.dumps(metrics,ensure_ascii=False,indent=2))
