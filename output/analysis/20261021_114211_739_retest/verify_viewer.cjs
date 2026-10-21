// Logic-only verification in a sandbox, without opening a browser or using input devices.
const fs=require('fs'),vm=require('vm'),path=require('path');
const assert=require('assert');
const dir=__dirname,html=fs.readFileSync(path.join(dir,'index.html'),'utf8');
const scripts=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(x=>x[1]);
scripts.forEach(s=>new vm.Script(s));
const controls={},buttons=['iso','xy','xz','yz'].map(view=>({dataset:{view},addEventListener(type,fn){this.handler=fn;}}));
let data,layout; const updates=[];
const ctx={window:{},document:{getElementById(id){return controls[id]??=( {addEventListener(type,fn){this.handler=fn;},textContent:''});},querySelectorAll(){return buttons;}},Plotly:{newPlot(id,d,l){data=d;layout=l;return Promise.resolve();},restyle(id,v,indices){for(let j=0;j<indices.length;j++){for(const [key,value]of Object.entries(v))data[indices[j]][key]=Array.isArray(value)?value[j]:value;}return Promise.resolve();},relayout(id,v){updates.push(v);return Promise.resolve();}}};
vm.createContext(ctx);vm.runInContext(scripts.at(-1),ctx);
(async()=>{
  await ctx.window.viewerReady;
  assert.deepStrictEqual(Array.from(data,t=>t.x.length),[5838,7889,1441,1441,1368,1368,7889]);
  assert.strictEqual(data[2].visible,false); assert.strictEqual(data[4].visible,true);
  const unchanged=JSON.stringify(data.map(t=>[t.x,t.y,t.z]));
  for(const [id,fit,actual]of [['original-mode',2,4],['relocated-mode',3,5]]){
    for(const mode of ['fit','points','both','actual']){
      await controls[id].handler({target:{value:mode}});
      assert.strictEqual(data[fit].visible,mode==='fit'||mode==='both');
      assert.strictEqual(data[actual].visible,mode!=='fit');
      assert.strictEqual(data[actual].mode,mode==='points'?'markers':'lines+markers');
    }
  }
  for(const value of ['1','5','10','20','40']) controls.zscale.handler({target:{value}});
  for(const button of buttons)button.handler();
  assert.strictEqual(JSON.stringify(data.map(t=>[t.x,t.y,t.z])),unchanged);
  const hashes=JSON.parse(fs.readFileSync(path.join(dir,'input_hashes.json'),'utf8'));
  const crypto=require('crypto'),root=path.resolve(dir,'../../..');
  for(const [p,hash]of Object.entries(hashes))assert.strictEqual(crypto.createHash('sha256').update(fs.readFileSync(path.join(root,p))).digest('hex'),hash,p);
  const result={numeric_replay_assertions:'passed',input_hashes_verified:Object.keys(hashes).length,viewer_javascript_syntax:'passed',viewer_mocked_control_handlers:'passed',all_trajectory_coordinates_unchanged_after_zscale:'passed',actual_browser_ui:'not performed; user prohibits Computer Use',mouse_and_computer_use:'not used',production_build_and_tests:'not run; read-only analysis'};
  fs.writeFileSync(path.join(dir,'verification.json'),JSON.stringify(result,null,2),'utf8');
  process.stdout.write(JSON.stringify(result,null,2));
})().catch(e=>{process.stderr.write(String(e));process.exitCode=1;});
