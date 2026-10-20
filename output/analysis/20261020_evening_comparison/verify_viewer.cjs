const {chromium}=require('playwright');
const path=require('path');const fs=require('fs');
(async()=>{const browser=await chromium.launch({headless:true,channel:'chrome',args:['--enable-webgl','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1500,height:1050},deviceScaleFactor:1});const errors=[];page.on('pageerror',e=>errors.push(e.message));
const checks=[];
for(const id of ['20261020_202202_369','20261020_204102_195','20261020_214257_216']){
 await page.goto('file://'+path.join(__dirname,id,'index.html'));await page.evaluate(()=>window.viewerReady);await page.waitForTimeout(1200);
 const initial=await page.evaluate(()=>{const c=document.getElementById('chart');return {counts:c.data.map(t=>t.x.length),visible:c.data.map(t=>t.visible),modes:c.data.map(t=>t.mode),coordinates:c.data.map(t=>[t.x[0],t.y[0],t.z[0]])}});
 if(!initial.visible[4]||!initial.visible[5]||initial.visible[2]||initial.visible[3])throw Error('wrong default references');
 for(const view of ['xy','xz','yz','iso']){await page.click(`[data-view="${view}"]`);await page.waitForTimeout(80)}
 for(const select of ['original-mode','relocated-mode'])for(const mode of ['points','fit','both','actual']){await page.selectOption('#'+select,mode);await page.waitForTimeout(60);const idx=select==='original-mode'?4:5;const fit=idx-2;const state=await page.evaluate(([i,j])=>({a:document.getElementById('chart').data[i].visible,f:document.getElementById('chart').data[j].visible,mode:document.getElementById('chart').data[i].mode}),[idx,fit]);if(state.a!==(mode!=='fit')||state.f!==(['fit','both'].includes(mode)))throw Error('reference switch failed')}
 for(const scale of ['1','5','10','40','20']){await page.selectOption('#zscale',scale);await page.waitForTimeout(60)}
 const after=await page.evaluate(()=>document.getElementById('chart').data.map(t=>[t.x[0],t.y[0],t.z[0]]));if(JSON.stringify(initial.coordinates)!==JSON.stringify(after))throw Error('coordinates changed');
 const before=await page.evaluate(()=>document.getElementById('chart')._fullLayout.scene._scene.getCamera());
 await page.mouse.move(760,610);await page.mouse.down();await page.mouse.move(940,700,{steps:12});await page.mouse.up();await page.waitForTimeout(200);
 const rotated=await page.evaluate(()=>document.getElementById('chart')._fullLayout.scene._scene.getCamera());if(JSON.stringify(before.eye)===JSON.stringify(rotated.eye))throw Error('rotation failed');
 const imageBeforeZoom=await page.locator('#chart').screenshot();await page.mouse.move(750,650);await page.mouse.wheel(0,-600);await page.waitForTimeout(1000);const zoomed=await page.evaluate(()=>document.getElementById('chart')._fullLayout.scene._scene.getCamera());const imageAfterZoom=await page.locator('#chart').screenshot();if(imageBeforeZoom.equals(imageAfterZoom))throw Error('zoom image unchanged');
 await page.mouse.move(760,610);await page.mouse.down({button:'right'});await page.mouse.move(820,650,{steps:8});await page.mouse.up({button:'right'});await page.waitForTimeout(200);
 const panned=await page.evaluate(()=>document.getElementById('chart')._fullLayout.scene._scene.getCamera());
 const panChanged=JSON.stringify(zoomed.center)!==JSON.stringify(panned.center);if(!panChanged)throw Error('pan failed');
 await page.click('[data-view="iso"]');await page.waitForTimeout(200);
 await page.screenshot({path:path.join(__dirname,id,'preview.png'),fullPage:true});
 checks.push({session:id,initial,views:true,independent_reference_modes:true,zscale_readouts_unchanged:true,rotation:true,wheel_zoom:true,right_drag_pan:panChanged});
}
await page.goto('file://'+path.join(__dirname,'report.html'));await page.screenshot({path:path.join(__dirname,'report_preview.png'),fullPage:false});
fs.writeFileSync(path.join(__dirname,'verification.json'),JSON.stringify({checks,errors},null,2));await browser.close();if(errors.length)throw Error(errors.join('\n'));console.log('Verified all three viewers, controls, rotation, zoom, pan; no page errors.');})();
