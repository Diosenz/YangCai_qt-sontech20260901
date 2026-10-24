/* 离线方案原型。输入几何证据与真值分离；不得用于生产控制。 */
const CONFIG = {maxSeconds:.9, maxDistance:2.0, confirmation:3, transitionPixel:2, identityGate:8};
const norm=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const initial=()=>({anchor:null, output:null, mode:'不可用', lastSim:null, revision:null,
  simShift:[0,0], streak:0, previousId:null, lastCandidate:null, budget:null});
function advance(old, frame, config=CONFIG) {
  const s=JSON.parse(JSON.stringify(old));
  const fresh=frame.id!==s.previousId;
  if (!fresh) return {...s, reason:'重复曝光，未计入重捕获确认'};
  s.previousId=frame.id;
  // 版本变化只续接参考坐标；不得解释为工件的真实瞬时位移。
  if (frame.sim && s.lastSim && frame.revision!==s.revision) {
    s.simShift=[s.simShift[0]+s.lastSim[0]-frame.sim[0],s.simShift[1]+s.lastSim[1]-frame.sim[1]];
  }
  s.lastSim=frame.sim; s.revision=frame.revision;
  const relativeSim=frame.sim ? frame.sim.map((x,j)=>x+s.simShift[j]) : null;
  const predicted=s.anchor && relativeSim && s.anchor.sim ?
    s.anchor.point.map((x,j)=>x+relativeSim[j]-s.anchor.sim[j]) : null;
  s.budget=s.anchor ? {seconds:frame.t-s.anchor.t,distance:Math.abs(frame.s-s.anchor.s)} : null;
  const within=s.budget && s.budget.seconds < config.maxSeconds-1e-9 && s.budget.distance < config.maxDistance-1e-9;
  const geom=frame.geometry;
  const identity=geom && (!predicted || norm(geom,predicted)<=config.identityGate);
  const continuous=identity && (!s.lastCandidate || norm(geom,s.lastCandidate)<=config.identityGate);
  s.streak=continuous ? s.streak+1 : 0;
  s.lastCandidate=identity ? geom : null;
  if (identity && (s.mode==='几何实测' || s.streak>=config.confirmation)) {
    if (s.output && norm(s.output,geom)>config.transitionPixel) {
      const length=norm(s.output,geom);
      s.output=s.output.map((x,j)=>x+(geom[j]-x)*config.transitionPixel/length);
      s.mode='恢复过渡'; s.reason='新帧确认后有界恢复；过渡点不是原始实测';
    } else {
      s.output=geom.slice(); s.mode='几何实测';
      s.anchor={point:geom.slice(),sim:relativeSim,t:frame.t,s:frame.s,id:frame.id};
      s.budget={seconds:0,distance:0}; s.reason='中心可见且双侧母材几何成立，刷新可靠锚';
    }
  } else if (s.anchor && predicted && within) {
    s.output=predicted;
    if (!geom && !frame.ambiguous && frame.partial && norm(frame.partial,predicted)<config.identityGate) {
      s.output=frame.partial.slice(); s.mode='母材辅助估计';
      s.reason='中心不可见，母材外推只支持估计；不刷新锚与预算';
    } else if (!geom && !frame.ambiguous && frame.singleLine) {
      const [a,b]=frame.singleLine;
      const delta=(a*predicted[0]-predicted[1]+b)/(a*a+1);
      s.output=[predicted[0]-a*delta,predicted[1]+delta]; s.mode='单侧约束估计';
      s.reason='仅投影到可见母材延长线；沿线方向仍不可观测';
    } else {
      s.mode='有限预测';
      s.reason=frame.ambiguous ? '多个完整候选，拒绝按亮度切换；保留有限预测' :
        identity ? `恢复确认 ${s.streak}/${config.confirmation}；不提前刷新锚` : '缺少完整原图几何；由旧锚与参考相对运动预测';
    }
  } else {
    s.output=null; s.mode='不可用';
    s.reason=!s.anchor ? `无可靠锚，原图确认 ${s.streak}/${config.confirmation}` :
      !predicted ? '续接参考缺失，不能可靠预测' : '时间或沿缝行程预算到限，停止输出';
  }
  return s;
}
function replay(sequence,config=CONFIG) {
  let s=initial();
  return sequence.frames.map(f=>{
    s=advance(s,f,config);
    return {id:f.id,t:f.t,s:f.s,mode:s.mode,output:s.output,rawGeometry:f.geometry,
      truth:f.truth,errorPixel:s.output ? norm(s.output,f.truth) : null,
      baseline:f.baseline || null,
      independent:s.mode==='几何实测',anchor:s.anchor,budget:s.budget,
      streak:s.streak,reason:s.reason,kind:f.kind};
  });
}
if(typeof module!=='undefined' && require.main===module) {
  const data=JSON.parse(require('fs').readFileSync(process.argv[2],'utf8'));
  const result=data.sequences.map(q=>({name:q.name,frames:replay(q)}));
  const summary=result.map(q=>{
    const valid=q.frames.filter(f=>f.errorPixel!==null), measured=valid.filter(f=>f.independent);
    return {name:q.name,frames:q.frames.length,available:valid.length,measured:measured.length,
      maxPixelError:Math.max(...valid.map(f=>f.errorPixel)),
      measuredMaxPixelError:measured.length ? Math.max(...measured.map(f=>f.errorPixel)) : null,
      baselineScope:'生产精定位单帧，不包括完整 KCF 跟踪、门控和控制链',
      baselineAvailable:q.frames.filter(f=>f.baseline?.point).length,
      baselineMaxPixelError:Math.max(0,...q.frames.map(f=>f.baseline?.errorPixel??0)),
      states:[...new Set(q.frames.map(f=>f.mode))]};
  });
  process.stdout.write(JSON.stringify({scope:'合成图像提取与方案回放，不是现场精度验收',config:CONFIG,summary,result},null,2));
}
