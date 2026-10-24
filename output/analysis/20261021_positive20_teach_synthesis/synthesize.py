"""Copy relevant historical W/T evidence, synthesize a circle, and re-evaluate saved data."""
from pathlib import Path
import ast,hashlib,html,importlib.util,json,shutil,sys
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
from PIL import Image,ImageDraw,ImageFont
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
PREV=ROOT/'output/analysis/20261021_152328_050_retest'
spec=importlib.util.spec_from_file_location('g',ROOT/'output/analysis/20261020_evening_comparison/assets/geometry.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
HASHES={}
def tracked(p):
    p=Path(p);HASHES[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();return p
def dump(name,value):
    (OUT/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,default=g.serializable),encoding='utf-8')
inventory=pd.read_csv(tracked(OUT/'all_chufeng_W_T_inventory.csv'))
old=json.loads(tracked(PREV/'results.json').read_text(encoding='utf-8'))
wc=old['original_circle'];wc={k:np.array(v) if isinstance(v,list) else v for k,v in wc.items()}
kc=old['raw_kcf_circle'];kc={k:np.array(v) if isinstance(v,list) else v for k,v in kc.items()}
source_paths=[Path('E:/SontechWorks/SWv1/data/Robotleadingdata/WorkPiece/20230608CHUFENGKOU_W.txt'),Path('E:/SontechWorks-new/SontechWorks_Weld_Merge/SWv1/data/Robotleadingdata/WorkPiece/20250606CHUFENGKOU_W.txt'),Path('E:/SontechWorks-new/SontechWorks_Weld_Merge/SWv1/data/Robotleadingdata/WorkPiece/20260308CHUFENGKOU_W.txt'),ROOT/'data/WorkPiece/20260404CHUFENGKOU1_W.txt']
sources=[];geometry_hashes=set();aligned=[]
for p in source_paths:
    a=np.loadtxt(tracked(p),ndmin=2);q=a[:,:3]
    gh=hashlib.sha256(np.round(q,6).astype('<f8').tobytes()).hexdigest()
    assert gh not in geometry_hashes;geometry_hashes.add(gh)
    c=g.circle(g.representatives(q,g.circle(q)['center']));transform=g.circle_pose_alignment(c,wc)
    canonical=q@transform['rotation'].T+transform['translation']
    assert abs(np.linalg.det(transform['rotation'])-1)<1e-10
    aligned.append(pd.DataFrame(dict(source=p.stem,x_mm=canonical[:,0],y_mm=canonical[:,1],z_mm=canonical[:,2])))
    pair=p.with_name(p.name[:-6]+'_T.txt');assert pair.exists(),pair
    tp=np.loadtxt(tracked(pair),ndmin=2);tq=tp[:,:3]
    nearest=np.linalg.norm(tq[:,None,:]-q[None,:,:],axis=2).min(axis=1)
    destination=OUT/'sources'/p.stem;destination.mkdir(parents=True,exist_ok=True)
    for source in [p,pair]:
        copied=destination/source.name
        if copied.exists():assert copied.read_bytes()==source.read_bytes()
        else:shutil.copy2(source,copied)
        assert hashlib.sha256(copied.read_bytes()).hexdigest()==HASHES[str(source)]
    aliases=inventory[inventory.geometry_sha256.eq(gh)&inventory.kind.eq('W')].path.tolist()
    sources.append(dict(name=p.stem[:-2],W_file=str(p),T_file=str(pair),W_points=len(q),T_points=len(tp),circle=c,weight=.25,geometry_sha256=gh,aliases=aliases,T_to_W_max_mm=float(nearest.max()),canonical_transform=transform))
# Four distinct teaching geometries get equal weight; file copies and recording
# density never increase weight. TCP/KCF are excluded from the synthesized radius.
radius=float(np.mean([a['circle']['radius_mm'] for a in sources]))
synthetic=dict(center=wc['center'],normal=wc['normal'],radius_mm=radius)
align=g.circle_pose_alignment(synthetic,kc)
current=dict(center=align['rotation']@synthetic['center']+align['translation'],normal=align['rotation']@synthetic['normal'],radius_mm=radius)
assert np.allclose(current['center'],kc['center'],atol=1e-10)
pd.concat(aligned,ignore_index=True).to_csv(OUT/'historical_teachings_in_common_frame.csv',index=False,encoding='utf-8-sig')
work=np.loadtxt(tracked(ROOT/'data/WorkPiece/20260404CHUFENGKOU1_W.txt'))
teach=np.loadtxt(tracked(ROOT/'data/WorkPiece/20260404CHUFENGKOU1_T.txt'))
def project_to_circle(q,c):
    n=c['normal'];delta=q-c['center'];plane=delta-(delta@n)[:,None]*n
    norm=np.linalg.norm(plane,axis=1);assert (norm>0).all()
    return c['center']+c['radius_mm']*plane/norm[:,None]
neww=work.copy();newt=teach.copy();neww[:,:3]=project_to_circle(work[:,:3],synthetic);newt[:,:3]=project_to_circle(teach[:,:3],synthetic)
base='20261021CHUFENGKOU20_SYNTH_ANALYSIS'
np.savetxt(OUT/(base+'_W.txt'),neww,fmt=['%.9f']*3+['%.4f']*5+['%.6f'],encoding='utf-8')
np.savetxt(OUT/(base+'_T.txt'),newt,fmt=['%.9f']*3+['%.4f']*5+['%.6f'],encoding='utf-8')
assert np.max(g.circle_distance(neww[:,:3],synthetic))<1e-10
assert np.max(abs(neww[:,3:]-work[:,3:]))==0 and np.max(abs(newt[:,3:]-teach[:,3:]))==0
np.savetxt(OUT/'teach_to_current_transform.csv',np.block([[align['rotation'],align['translation'][:,None]],[np.zeros((1,3)),np.ones((1,1))]]),delimiter=',',fmt='%.12f')
metrics={};data={};comparison=[];verification={}
for label,file in [('TCP','tcp_material_frame.csv'),('raw_KCF','raw_kcf_material_frame.csv'),('final_vision','final_vision_material_frame.csv')]:
    frame=pd.read_csv(tracked(PREV/file));q=frame.iloc[:,1:4].to_numpy();e=g.circle_distance(q,current)
    foot=project_to_circle(q,current)
    assert np.max(abs(np.linalg.norm(q-foot,axis=1)-e))<1e-10
    a=g.metric(e);imax=int(e.argmax())
    a.update(within_2mm_fraction=float(np.mean(e<=2)),within_3mm_fraction=float(np.mean(e<=3)),above_3mm_count=int(np.sum(e>3)),max_time=pd.to_datetime(frame.timestamp_ms.iloc[imax],unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3])
    frame['distance_mm']=e
    for j,axis in enumerate('xyz'):frame['foot_'+axis+'_mm']=foot[:,j]
    frame.to_csv(OUT/(label+'_to_synthesized_circle.csv'),index=False,encoding='utf-8-sig')
    metrics[label]=a;data[label]=(frame.timestamp_ms.to_numpy(),q,e)
    previous=old['metrics'][label+'_to_moved_teach_circle']
    comparison.extend([dict(trajectory=label,reference='previous_single_teach_circle',**previous),dict(trajectory=label,reference='four_teachings_equal_weight_circle',**a)])
# Evidence sensitivity is specified before looking at run errors; do not select
# the best-performing radius from this table.
sensitivity=[]
for omitted in range(4):
    rad=float(np.mean([a['circle']['radius_mm'] for i,a in enumerate(sources) if i!=omitted]));c=dict(current,radius_mm=rad)
    for label in ['TCP','raw_KCF']:sensitivity.append(dict(omitted_source=sources[omitted]['name'],radius_mm=rad,trajectory=label,**g.metric(g.circle_distance(data[label][1],c))))
pd.DataFrame(sensitivity).to_csv(OUT/'leave_one_teaching_out_sensitivity.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(comparison).to_csv(OUT/'reference_comparison.csv',index=False,encoding='utf-8-sig')
pd.DataFrame([dict(trajectory=k,**v) for k,v in metrics.items()]).to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
# Classification is explicit; unmapped full circles are retained in the inventory.
classification=[]
for row in inventory[inventory.kind.eq('W')].drop_duplicates('geometry_sha256').to_dict('records'):
    name=row['name'];chosen=row['geometry_sha256'] in geometry_hashes
    reason='明确正20mm映射；按几何去重后纳入' if chosen else ('反面/FLIP；排除' if row['flip'] else '无明确正20mm映射或仅不完整圆弧；仅保留检索结果，不进入主综合')
    classification.append(dict(name=name,path=row['path'],included=chosen,reason=reason,radius_mm=row.get('radius_mm'),coverage_deg=row.get('coverage_deg')))
pd.DataFrame(classification).to_csv(OUT/'selection_decisions.csv',index=False,encoding='utf-8-sig')
result=dict(session=old['session'],method='Four source teachings with explicit positive-20mm mappings; deduplicate by XYZ geometry; fit each circle using existing angle-balanced method; rigidly normalize centers/normals; equal source weights; arithmetic mean radius. Preserve current teaching pose/axis/time columns as unvalidated analysis metadata.',sources=sources,new_original_circle=synthetic,current_circle=current,alignment=align,metrics=metrics,previous_reference_radius_mm=wc['radius_mm'],new_reference_radius_mm=radius,source_radius_range_mm=[min(a['circle']['radius_mm'] for a in sources),max(a['circle']['radius_mm'] for a in sources)],coordinate_frame=old['coordinate_frame'],reference_status='Composite historical teaching reference; current pose estimated from raw KCF; not independently measured truth for this specimen',generated_files=dict(W=base+'_W.txt',T=base+'_T.txt',runtime_configuration_changed=False),interpretation='Same TCP/KCF measurements. Error changes arise only from changed reference geometry; they are not an improvement caused by a new control algorithm.',selection_decisions=classification)
dump('results.json',result);dump('synthesized_original_circle.json',synthetic);dump('moved_synthesized_circle.json',current)
def circlepoints(c):
    n=c['normal'];u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u);v=np.cross(n,u);angle=np.linspace(0,2*np.pi,1441)
    return c['center']+c['radius_mm']*(np.cos(angle)[:,None]*u+np.sin(angle)[:,None]*v)
traces=[]
def trace(q,name,color,visible=True,mode='lines',dash='solid'):
    traces.append(dict(type='scatter3d',name=name,x=q[:,0].tolist(),y=q[:,1].tolist(),z=q[:,2].tolist(),visible=visible,mode=mode,line=dict(color=color,width=3,dash=dash),marker=dict(color=color,size=2)))
trace(data['TCP'][1],'TCP 实测轨迹','#0072b2');trace(data['raw_KCF'][1],'KCF 融合前轨迹','#d55e00')
originalcurve=circlepoints(synthetic);movedcurve=circlepoints(current)
trace(originalcurve,'综合后的新原始示教拟合圆','#777777',dash='dash');trace(movedcurve,'移动后的综合示教拟合圆','#009e73')
trace(neww[:,:3],'新原始示教结果点','#777777',False,'lines+markers','dash');trace(neww[:,:3]@align['rotation'].T+align['translation'],'移动后新示教结果点','#009e73',False,'lines+markers')
allq=np.vstack([np.c_[a['x'],a['y'],a['z']] for a in traces]);lo=allq.min(0);hi=allq.max(0);span=hi-lo
assets=ROOT/'output/analysis/20261020_evening_comparison/assets'
page=tracked(assets/'viewer_template.html').read_text(encoding='utf-8')
page=page.replace('/* PLOTLY_LIBRARY */',tracked(assets/'plotly-2.35.2.min.js').read_text(encoding='utf-8')).replace('/* TRACE_DATA */',json.dumps(traces,ensure_ascii=False)).replace('/* AXIS_RANGES */',json.dumps(np.c_[lo-span*.06,hi+span*.06].tolist())).replace('/* AXIS_SPANS */',json.dumps(span.tolist()))
page=page.replace('20261019_110509_938 · 固定变位机基坐标系 · 统一 S1/S2 = COMMON_POSE','20261021_152328_050 · 四组历史正20mm示教综合 · 变位机末节材料系')
i=page.index('<footer>');j=page.index('</footer>',i)
page=page[:i]+'<footer><a href="report.html">返回综合分析报告</a> · 4组历史示教按XYZ去重、每组等权，新原始圆半径 '+f'{radius:.6f}'+' mm。尺寸由历史示教确定，当前KCF仅用于对齐圆心/法线。原始TCP/KCF不拟合、不删点。误差按空间连续圆解析计算，不受显示采样或Z倍率影响。该参考是历史综合估计，不是当前工件的独立真值。未使用鼠标，浏览器真实交互未验收。</footer>'+page[j+len('</footer>'):]
(OUT/'index.html').write_text(page,encoding='utf-8');dump('viewer_data.json',dict(traces=traces,ranges=np.c_[lo-span*.06,hi+span*.06].tolist(),spans=span))
# Static preview uses the established local chart renderer.
im=Image.new('RGB',(1800,1250),'white');draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',24);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',19)
tree=ast.parse(tracked(PREV/'analyze.py').read_text(encoding='utf-8-sig'));fn=next(a for a in tree.body if isinstance(a,ast.FunctionDef) and a.name=='chart');exec(compile(ast.Module(body=[fn],type_ignores=[]),'chart','exec'))
draw.text((50,15),'2026-10-21 复测：四组历史原始示教综合后重新评价',fill='#203345',font=font)
draw.text((50,55),f'新原始圆半径 {radius:.6f} mm；历史每组等权；TCP/KCF数据未变；误差变化源于参考变化',fill='#526576',font=small)
colors=['#777777','#0072b2','#d55e00','#009e73'];labels=['综合新原始示教圆','TCP 实测','KCF 融合前','移动后综合示教圆']
for i,(color,label) in enumerate(zip(colors,labels)):draw.text((50+i*420,95),label,fill=color,font=small)
curves=[originalcurve,data['TCP'][1],data['raw_KCF'][1],movedcurve]
for box,(i,j,title) in zip([(125,225,850,625),(1025,225,1750,625)],[(0,1,'XY 投影（等比例）'),(0,2,'XZ 投影（等比例）')]):chart(box,[(q[:,i],q[:,j],c) for q,c in zip(curves,colors)],title,'XYZ'[i]+' (mm)','XYZ'[j]+' (mm)',True)
start=min(data['TCP'][0][0],data['raw_KCF'][0][0])
chart((125,825,850,1165),[((data[k][0]-start)/1000,data[k][2],c) for k,c in [('TCP',colors[1]),('raw_KCF',colors[2])]],'全部有效点到综合圆的误差','在线时间 (s)','三维距离 (mm)')
chart((1025,825,1750,1165),[(np.sort(data[k][2]),np.arange(1,len(data[k][2])+1)/len(data[k][2]),c) for k,c in [('TCP',colors[1]),('raw_KCF',colors[2])]],'误差累计分布','距离 (mm)','累计比例');im.save(OUT/'preview.png')
def table(rows,cols):
    return '<table><tr>'+''.join('<th>'+label+'</th>' for key,label in cols)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(f'{a[key]:.4f}' if isinstance(a[key],float) else str(a[key]))+'</td>' for key,label in cols)+'</tr>' for a in rows)+'</table>'
source_table=table([dict(name=a['name'],n=a['W_points'],tn=a['T_points'],radius=a['circle']['radius_mm'],copies=len(a['aliases']),Tdist=a['T_to_W_max_mm']) for a in sources],[('name','原始示教组'),('n','W点数'),('tn','T点数'),('radius','拟合半径/mm'),('copies','备份W条目数'),('Tdist','T点到本组W最近点最大距离/mm')])
metric_table=table([dict(name=k,**v,within3=f"{100*v['within_3mm_fraction']:.2f}%") for k,v in metrics.items()],[('name','轨迹'),('n','点数'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm'),('p95_mm','P95/mm'),('within3','≤3mm比例'),('max_time','最大值时间')])
comparison_table=table(comparison,[('trajectory','轨迹'),('reference','参考'),('mean_mm','平均/mm'),('max_mm','最大/mm'),('rmse_mm','RMSE/mm')])
report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>正20mm历史示教综合与复测</title><style>body{{max-width:1250px;margin:35px auto;padding:0 24px;font:16px/1.8 system-ui,"Microsoft YaHei";color:#203345}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{border-bottom:1px solid #dce5ec;padding:9px;text-align:left}}th{{background:#edf4fa}}.note{{background:#eaf5f0;padding:15px}}img{{max-width:100%}}a{{color:#0072b2}}</style>
<h1>出风口小（正20mm）：历史原始示教综合与误差重算</h1><p>复测会话 20261021_152328_050。<a href="index.html">四轨迹交互三维图</a> · <a href="metrics.csv">本次数值表</a> · <a href="results.json">完整结果</a></p>
<p class="note">已只读检索用户指定的8个SontechWorks备份根目录，找到210个出风口相关W/T条目、25种不同W几何（包括反面与早期数据）。筛选有正20mm映射依据的4种原始示教几何，去重后每组等权，形成新原始示教圆：<b>R={radius:.6f} mm</b>。随后用同一轮TCP、KCF数据重新计算点到移动后综合圆的最短三维距离。</p>
<h2>1. 对应程序与示教文件</h2><p>当前Qt工程配置 <code>config/workpieces/workpiece_selection.txt:19</code> 和本轮现场日志均对应：反向扫描 <b>20260914CFKONTEST0</b>，GlobalCorrection <code>20260914CFKONTEST</code>，IBVSStart <code>20260405CHUFENGKOULIUCHENGTEST</code>；轨迹基名是 <code>20260404CHUFENGKOU1</code>。它们是不同用途的名字，不能把控制柜程序名当成W/T文件基名。最新SontechWorks分支的正面入口映射到 <code>20260308CHUFENGKOU_W/T.txt</code>，SontechWorks_IBVS映射到 <code>20250606CHUFENGKOU_W/T.txt</code>，Weld_Merge映射到当前 <code>20260404CHUFENGKOU1_W/T.txt</code>；2023历史代码也有 <code>20230608CHUFENGKOU</code> 的正面工件语义证据。</p>
<p>本轮按用户最新范围，仅收集原始示教结果W/T，不将JBR脉冲点纳入几何。<a href="all_chufeng_W_T_inventory.csv">完整文件清单</a> · <a href="historical_mappings.json">各分支源码映射摘录</a> · <a href="selection_decisions.csv">逐组纳入/排除理由</a>。检索中1份反面FLIP文件解析失败，已记录并排除，不影响4组主数据。</p>
<h2>2. 纳入的四组独立文件内容</h2>{source_table}<p>同名或异名但XYZ完全相同的文件只计一组；<code>20260404CHUFENGKOU</code> 与 <code>20260404CHUFENGKOU1</code> 的W几何相同，不重复加权。T是稀疏示教模板，部分T点已被W覆盖，且有些T只有段首/段末两个点，不能作为另一份完整圆观测重复计数。本次T用于配对、来源核查和保留当前姿态模板；每组的圆几何由其完整W轨迹确定。实际采用的8份W/T已逐字节复制在 <code>sources/</code> 子目录。</p>
<p>反面FLIP/FAN、2021年不完整圆弧、以及没有明确正20mm映射的TRUEVALUE/其它日期候选，不进入主综合。特别是 <code>20260309CHUFENGKOU</code>、<code>20260328CHUFENGKOUTRUEVALUE</code>、<code>20260405CHUFENGKOUTRUEVALUE</code> 虽找到完整文件，但没有把“名字相似”直接升级为当前工件映射证据。它们仍完整列在清单中，未删除。</p>
<h2>3. 怎样综合成新的原始示教</h2><p>每组沿用上一轮2°角度分箱代表点的三维圆拟合方法。先对各组做圆心/法线刚体归一，放到当前原始示教的材料坐标系，消除各次摆放位置差异；不缩放。综合半径采用四组拟合半径的算术平均，每组权重1/4。它等价于最小化四个半径的等权平方差；不按备份数量或采样点数加权，也不依据本轮TCP/KCF误差挑半径。四组半径范围为 {result['source_radius_range_mm'][0]:.3f}～{result['source_radius_range_mm'][1]:.3f} mm，因此这是历史综合参考，不宣称所有记录来自同一件实物或同一次标定。</p>
<p>新原始圆中心/法线保留当前原始示教的参考摆放。新W/T将当前模板XYZ投到综合圆上，分别保留{len(neww)}点和{len(newt)}点；原来的姿态、外轴和时间列保留为分析元信息，未重新标定或验证为机器人可执行程序。文件专门另存于本分析目录，<b>未替换生产W/T、未改工件配置、未发送运动命令</b>。<a href="{base}_W.txt">新原始示教_W</a> · <a href="{base}_T.txt">新原始示教_T</a> · <a href="synthesized_original_circle.json">新原始圆参数</a> · <a href="historical_teachings_in_common_frame.csv">四组归一后的原始点</a>。</p>
<p>移动到本轮工件位置仍使用原始KCF圆心/法线估计，保留综合半径 {radius:.6f} mm，不把半径改为KCF的 {kc['radius_mm']:.6f} mm。<a href="teach_to_current_transform.csv">移动矩阵</a> · <a href="moved_synthesized_circle.json">移动后圆参数</a>。圆周几何不依赖绕自身法线的相位角。</p>
<h2>4. 新参考下的误差</h2>{metric_table}<p>e=sqrt((ρ−R)²+h²)，ρ为点在圆平面投影到圆心的半径，h为到圆平面的有符号高度。按空间连续圆解析计算，不使用绘图圆的离散折线。沿用同一坐标系、同一筛选与同一批有效点，没有对TCP/KCF拟合、平滑或删点。均值为样本加权，不评价沿缝时序或姿态角。</p>
<h2>5. 与上一版单组示教圆的对比</h2>{comparison_table}<p><b>这是参考几何变化，不是纠偏算法性能再次提升。</b>TCP、KCF原始测量完全未变；上一版只使用当前单组示教半径 {wc['radius_mm']:.6f} mm，本次使用4组综合半径 {radius:.6f} mm。报告误差随参考定义改变，应同时保留来源与对照。结合用户认可的约2～3 mm可接受量级，可按新表评价整体与局部峰值，不能把历史综合圆称为本件实物的独立真值。</p>
<p>另做“每次删去一组历史示教”的参考敏感性分析，见<a href="leave_one_teaching_out_sensitivity.csv">敏感性表</a>。主结果固定采用四组等权，不从敏感性表挑最小误差。</p>
<img src="preview.png" alt="新综合示教四轨迹和误差">
<h2>6. 验证和复现</h2><p>已核对4组XYZ哈希互异、各复制文件与源文件SHA-256相等、变换正交、生成W/T点位在新圆上、元信息列保持不变、全部误差与圆周垂足距离一致。所有输入哈希见<a href="input_hashes.json">清单</a>。复现先运行 <code>inventory.py</code>，再运行 <code>synthesize.py</code>。数值结果和来源状态见<a href="verification.json">验证记录</a>。未修改外部备份、当前生产源码/配置/原始数据，未删除文件、使用鼠标或发送设备指令；真实浏览器交互未验收。</p></html>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
for source in [ROOT/'config/workpieces/workpiece_selection.txt',OUT/'historical_mappings.json',OUT/'inventory_summary.json']:tracked(source)
for p,h in HASHES.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
dump('input_hashes.json',HASHES);dump('verification.json',dict(unique_source_geometries=len(geometry_hashes),source_weights_sum=sum(a['weight'] for a in sources),copied_source_hashes='passed',rigid_alignment='passed',generated_points_on_circle='passed',preserved_template_metadata='passed',all_point_analytic_circle_distance='passed',input_hashes_verified=len(HASHES),source_files_deleted=False,runtime_configuration_changed=False,mouse_used=False,actual_browser_ui='not performed'))
print(json.dumps(dict(radius_mm=radius,source_names=[a['name'] for a in sources],metrics=metrics),ensure_ascii=False,indent=2))
