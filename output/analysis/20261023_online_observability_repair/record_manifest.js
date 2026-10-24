const fs=require('fs'),crypto=require('crypto'),cp=require('child_process');
const root=process.cwd();
const backupRoot='.codex_backups/online_observability_main_20261023';
const baselines=new Map();
const normalized=p=>p.replace(/\\/g,'/');
for(const manifest of [backupRoot+'/manifest.json','.codex_backups/estimator_observability_20261023_120542/manifest.json','.codex_backups/detector_timed_recovery_20261023_120642/manifest.json']){
 for(const row of JSON.parse(fs.readFileSync(manifest,'utf8').replace(/^\uFEFF/,''))){
  const path=normalized(row.path); if(!baselines.has(path))baselines.set(path,{baselineSha256:row.sha256,backup:normalized(row.backup||manifest.slice(0,manifest.lastIndexOf('/'))+'/'+path)});
 }
}
for(const path of ['UI/include/OnlineLocalRegistrationStability.h','UI/src/UiController.cpp','tests/test_online_local_registration_stability.cpp']) {
 const backup=backupRoot+'/extra/'+path; if(!fs.existsSync(backup))throw new Error('backup missing '+backup);
 const b=fs.readFileSync(backup);baselines.set(path,{baselineSha256:crypto.createHash('sha256').update(b).digest('hex').toUpperCase(),backup});
}
const additions=['DigitalTwin/include/OnlineSeamCandidateGeometry.h','DigitalTwin/src/OnlineSeamCandidateGeometry.cpp','tests/test_online_seam_candidate_geometry.cpp','tests/test_online_seam_geometry_worker.cpp'];
const records=[];
for(const path of new Set([...baselines.keys(),...additions])){
 const b=fs.readFileSync(path);const hash=crypto.createHash('sha256').update(b).digest('hex').toUpperCase();const base=baselines.get(path);
 if(base&&hash===base.baselineSha256)continue;
 const text=b.toString('utf8');const cpp=/\.(cpp|h)$/.test(path);const bom=b.subarray(0,3).equals(Buffer.from([239,187,191]));const bareLf=(text.match(/(?<!\r)\n/g)||[]).length;
 if(cpp&&(!bom||bareLf||text.includes('\ufffd')))throw new Error('encoding issue '+path);
 records.push({path,status:base?'modified_from_task_backup':'added_this_task',...base,sha256:hash,bytes:b.length,utf8Bom:bom,bareLf});
}
const untouched=Array.from(baselines).filter(([p,b])=>!records.some(r=>r.path===p)).map(([path,b])=>({path,sha256:b.baselineSha256}));
const out={basis:'Task backups, not repository HEAD; existing dirty changes preserved.',changedFiles:records.length,files:records,backedUpButUnchanged:untouched};
fs.writeFileSync('output/analysis/20261023_online_observability_repair/changed_files.json',JSON.stringify(out,null,2)+'\n');
console.log(JSON.stringify({changedFiles:records.length,encoding:'all changed C++ UTF-8 BOM / CRLF / no replacement characters',unchanged:untouched.map(r=>r.path),paths:records.map(r=>r.path)},null,2));