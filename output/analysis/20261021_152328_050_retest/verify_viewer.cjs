// Check JavaScript and control logic with stubs. No browser or input devices.
const fs=require('fs'),vm=require('vm'),path=require('path'),assert=require('assert');
const root=__dirname,html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const scripts=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(x=>x[1]);
scripts.forEach(s=>new vm.Script(s));
const controls={},buttons=['iso','xy','xz','yz'].map(view=>({dataset:{view},addEventListener(type,fn){this.handler=fn;}}));
let data,layout;const updates=[];
const context={window:{},document:{getElementById(id){return controls[id]??={addEventListener(type,fn){this.handler=fn;},textContent:''};},querySelectorAll(){return buttons;}},Plotly:{newPlot(id,d,l){data=d;layout=l;return Promise.resolve();},restyle(id,values,indices){for(let j=0;j<indices.length;j++)for(const[key,value]of Object.entries(values))data[indices[j]][key]=Array.isArray(value)?value[j]:value;return Promise.resolve();},relayout(id,values){updates.push(values);return Promise.resolve();}}};
vm.createContext(context);vm.runInContext(scripts.at(-1),context);
(async()=>{
 await context.window.viewerReady;
 assert.deepStrictEqual(Array.from(data,t=>t.x.length),[5848,7522,1441,1441,1368,1368]);
 assert.strictEqual(data[2].visible,false);assert.strictEqual(data[3].visible,false);
 assert.strictEqual(data[4].visible,true);assert.strictEqual(data[5].visible,true);
 const snapshot=JSON.stringify(data.map(t=>[t.x,t.y,t.z]));
 for(const[id,fit,actual]of[['original-mode',2,4],['relocated-mode',3,5]]){
  const other=actual===4?5:4,prior=JSON.stringify(data[other]);
  for(const value of ['fit','points','both','actual']){
   await controls[id].handler({target:{value}});
   assert.strictEqual(data[fit].visible,value==='fit'||value==='both');
   assert.strictEqual(data[actual].visible,value!=='fit');
   assert.strictEqual(data[actual].mode,value==='points'?'markers':'lines+markers');
   assert.strictEqual(JSON.stringify(data[other]),prior);
  }
 }
 for(const value of ['1','5','10','20','40']){
  controls.zscale.handler({target:{value}});
  const update=updates.at(-1)['scene.aspectratio'];
  assert(Math.abs(update.z/layout.scene.aspectratio.z-Number(value)/20)<1e-12);
 }
 for(const button of buttons)button.handler();
 assert.strictEqual(JSON.stringify(data.map(t=>[t.x,t.y,t.z])),snapshot);
 const result={javascript_syntax:'passed',stubbed_control_handlers:'passed',independent_teach_switches:'passed',zscale_does_not_change_coordinates:'passed',view_buttons:'passed',actual_browser_ui:'not performed',mouse:'not used'};
 fs.writeFileSync(path.join(root,'viewer_verification.json'),JSON.stringify(result,null,2),'utf8');
 process.stdout.write(JSON.stringify(result,null,2));
})().catch(e=>{process.stderr.write(e.stack);process.exitCode=1;});
