// Headless local-artifact rendering and DOM event checks. No mouse APIs.
const fs=require('fs'),path=require('path'),assert=require('assert');
const {pathToFileURL}=require('url');
const {chromium}=require('C:/Users/admin/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{
 const browser=await chromium.launch({executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless:true,args:['--allow-file-access-from-files']});
 const page=await browser.newPage({viewport:{width:1440,height:1080},deviceScaleFactor:1});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto(pathToFileURL(path.join(__dirname,'index.html')).href);
 await page.evaluate(()=>window.viewerReady);
 await page.waitForFunction(()=>document.querySelector('#plot .gl-container canvas'));
 const output=await page.evaluate(async()=>{
  const graph=document.getElementById('plot');const snapshot=JSON.stringify(graph.data.map(t=>[t.x,t.y,t.z]));
  const original=graph.layout.scene.aspectratio.z;
  const ranges=JSON.stringify(['x','y','z'].map(a=>graph.layout.scene[a+'axis'].range));
  for(const k of [5,10,20,40,1]){
   let s=document.getElementById('zscale');s.value=String(k);s.dispatchEvent(new Event('change'));
   await new Promise(r=>setTimeout(r,80));
   if(Math.abs(graph.layout.scene.aspectratio.z-original*k)>1e-10)throw Error('zscale mismatch');
  }
  for(const[id,fit,actual]of[['original-mode',3,5],['relocated-mode',4,6]]){
   for(const value of ['actual','points','both','fit']){
    let s=document.getElementById(id);s.value=value;s.dispatchEvent(new Event('change'));await new Promise(r=>setTimeout(r,80));
    if(graph.data[fit].visible!==(value==='fit'||value==='both')||graph.data[actual].visible!==(value!=='fit'))throw Error('visibility mismatch');
    if(graph.data[actual].mode!==(value==='points'?'markers':'lines+markers'))throw Error('mode mismatch');
   }
  }
  for(const button of document.querySelectorAll('[data-view]')){button.dispatchEvent(new Event('click'));await new Promise(r=>setTimeout(r,80));}
  document.querySelector('[data-view="iso"]').dispatchEvent(new Event('click'));
  await new Promise(r=>setTimeout(r,600));
  if(JSON.stringify(['x','y','z'].map(a=>graph.layout.scene[a+'axis'].range))!==ranges)throw Error('axis ranges changed');
  if(JSON.stringify(graph.data.map(t=>[t.x,t.y,t.z]))!==snapshot)throw Error('coordinates changed');
  return {trace_counts:graph.data.map(t=>t.x.length),zscale_and_independent_selectors:'passed',view_presets:'passed',coordinate_immutability:'passed'};
 });
 await page.screenshot({path:path.join(__dirname,'browser_3d_preview.png'),fullPage:true});
 await page.goto(pathToFileURL(path.join(__dirname,'axis_fluctuations.html')).href);
 await page.evaluate(()=>window.axisViewerReady);
 const axes=await page.evaluate(()=>({traces:document.getElementById('axes').data.length,canvas:document.querySelectorAll('canvas').length}));
 assert.strictEqual(axes.traces,9);assert(axes.canvas>0);
 await page.screenshot({path:path.join(__dirname,'browser_axes_preview.png'),fullPage:true});
 await page.goto(pathToFileURL(path.join(__dirname,'report.html')).href);
 await page.waitForFunction(()=>document.querySelectorAll('iframe').length===2);
 await page.screenshot({path:path.join(__dirname,'browser_report_preview.png'),fullPage:false});
 const report=await page.evaluate(()=>({title:document.title,overflow:document.documentElement.scrollWidth>innerWidth,iframes:[...document.querySelectorAll('iframe')].map(f=>f.getAttribute('src'))}));
 assert.strictEqual(report.overflow,false);
 await page.setViewportSize({width:430,height:900});
 await page.screenshot({path:path.join(__dirname,'browser_mobile_preview.png'),fullPage:false});
 const mobile=await page.evaluate(()=>({overflow:document.documentElement.scrollWidth>innerWidth}));assert.strictEqual(mobile.overflow,false);
 assert.deepStrictEqual(errors,[]);
 const result={status:'passed',browser:'Edge headless',local_3d_render:'passed',controls_verified_by:'DOM events, no mouse input',axes,report,mobile,errors,...output,mouse_used:false,manual_drag_zoom_pan:'not performed'};
 fs.writeFileSync(path.join(__dirname,'browser_verification.json'),JSON.stringify(result,null,2));
 process.stdout.write(JSON.stringify(result,null,2));await browser.close();
})().catch(e=>{process.stderr.write(e.stack);process.exitCode=1;});
