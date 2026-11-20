import {createFeedingScene} from './feeding-scene.js';
import {createKinematics} from './mouth-follow-core.mjs';
import {SmartFeeder,distance} from './smart-feeding-core.mjs';
import {startMouthCamera} from './mouth-follow-camera.mjs';
const $=id=>document.getElementById(id),fmt=p=>p.map(x=>(x*1000).toFixed(1)).join(' / ');
let engine,scene,latest=null,demo=null,closeCamera,alive=true,raf,lastUi=0,drawnId=null,lastDemoSample=0,recordingBusy=false,pendingVideo=false,videoUrl=null;
function stop(reason){demo=null;engine?.cancel(reason);$('demoStatus').textContent='';if(recordingBusy)void finishVideo(true);}
async function finishVideo(incomplete){
 if(!recordingBusy)return;recordingBusy=false;pendingVideo=true;
 try{const blob=await scene.finishRecording();if(!blob)throw Error('没有录制数据');if(videoUrl)URL.revokeObjectURL(videoUrl);videoUrl=URL.createObjectURL(blob);
  const link=$('videoDownload');link.href=videoUrl;link.download=`EAT-RDK-${engine.selected.id}${incomplete?'-incomplete':''}.${blob.type.includes('mp4')?'mp4':'webm'}`;link.hidden=false;link.textContent=incomplete?'下载未完成片段':'下载完整仿真视频';
  $('recordStatus').textContent=incomplete?'录制中断，已保留片段':'录制完成 · 仅含三维仿真，不含相机画面';
 }catch(e){$('recordStatus').textContent=`录制失败：${e.message}`;}finally{pendingVideo=false;}
}
function startDemo(record=false){
 if(!engine||pendingVideo)return;stop();engine.enableDemo();
 if(record){try{scene.beginRecording();recordingBusy=true;$('videoDownload').hidden=true;$('recordStatus').textContent='正在录制 1280 × 720 仿真画面';}catch(e){engine.cancel();$('recordStatus').textContent=e.message;return;}}
 demo={start:performance.now(),offset:offsets(),count:engine.triggerCount};lastDemoSample=0;
}

function offsets(){return ['dx','dy','dz'].map(id=>Number($(id).value)/1000);}
function selectVisual(){
 const plan=engine.run?.plan||engine.selected;
 if(drawnId===plan.id)return;drawnId=plan.id;
 scene.setLibraryTargets(engine.lib.trajectories.map(p=>p.virtual_mouth_m),engine.lib.trajectories.indexOf(plan));
 const path=plan.stages.flatMap(s=>s.samples.filter((_,i)=>i%4===0).map(s=>engine.kin.pose(s.q_rad).point));scene.setTrajectory(path);
 $('routeTitle').textContent=`${plan.id} · ${plan.duration_s.toFixed(1)} 秒完整喂饭`;
}
function updateUi(now){
 const plan=engine.run?.plan||engine.selected;
 $('mode').textContent=demo?'模拟视觉输入':engine.run?.source==='manual'?'手动仿真':engine.enabled?'张嘴触发已开启':'已暂停';
 $('runStatus').textContent=engine.reason;
 $('match').textContent=`${(distance(plan.virtual_mouth_m,engine.target)*1000).toFixed(1)} mm`;
 $('tilt').textContent=`${scene.tilt().toFixed(3)}°`;
 $('time').textContent=`${engine.elapsed.toFixed(1)} / ${plan.duration_s.toFixed(1)} s`;$('progress').value=engine.elapsed/plan.duration_s;
 $('targetXYZ').textContent=fmt(engine.target);
 $('freshness').textContent=latest?`${latest.ratio.toFixed(3)} / ${Math.round(now-latest.atMs)} ms`:'—';
 $('arm').disabled=!!demo||!!engine.run||engine.enabled||!engine.vision.ready(now)||distance(engine.q,engine.lib.home_q_rad)>1e-6;
 $('preview').disabled=!!engine.run||engine.enabled;
 $('demo').disabled=!!engine.run||engine.enabled||pendingVideo;$('record').disabled=$('demo').disabled;
 for(const id of ['dx','dy','dz'])$(id).disabled=!!engine.run||engine.enabled;
 $('ranking').innerHTML=engine.ranking.slice(0,5).map((r,i)=>`<tr><td>${i+1}</td><td>${r.plan.id}</td><td>${(r.distance_m*1000).toFixed(1)} mm</td><td>${r.plan.duration_s.toFixed(1)} s</td></tr>`).join('');
 $('matchStatus').textContent=engine.ranking[0].distance_m<=engine.lib.max_match_distance_m?'最近轨迹在 25 mm 内':'超过 25 mm · 不触发';
 $('selectedInfo').textContent=`${plan.id} 终点：${fmt(plan.virtual_mouth_m)} mm`;
 for(const button of $('routeGrid').children){button.classList.toggle('selected',button.dataset.id===plan.id);button.classList.toggle('nearest',button.dataset.id===engine.ranking[0].plan.id);button.disabled=!!engine.run||engine.enabled;}
}
function render(now){
 if(!alive)return;raf=requestAnimationFrame(render);if(!engine)return;
 if(demo&&now-lastDemoSample>=100){
  lastDemoSample=now;const t=(now-demo.start)/1000,off=demo.offset;
  const sample={xyz:[off[0]/.4,off[1]/.4,.65+off[2]/.4],ratio:t<.7?.03:.28,faces:1,frameId:`demo-${now}`,atMs:now};
  engine.ingest(sample,now);$('demoStatus').textContent=`模拟${t<.7?'闭嘴':'张嘴'} · 已触发 ${engine.triggerCount-demo.count} 次`;
  if(!engine.enabled&&!engine.run){demo=null;}
 }
 engine.tick(now);
 if(demo&&!engine.run&&engine.elapsed>=engine.selected.duration_s){if(recordingBusy)void finishVideo(false);stop('模拟张嘴演示完成 · 已选择并播放一条最近轨迹');engine.anchor=null;}
 if(recordingBusy&&!demo&&!engine.run)void finishVideo(true);
 scene.recordingFrame(`${engine.selected.id} · ${engine.reason}`,engine.elapsed,engine.selected.duration_s);
 selectVisual();scene.setMouth(engine.target);scene.draw(engine.q,engine.stage,engine.local||0,true);
 if(now-lastUi>100){updateUi(now);lastUi=now;}
}
$('arm').onclick=()=>{if(engine.enable(performance.now())){$('demoStatus').textContent='';engine.setTarget(engine.lib.anchor_m);}};
$('stop').onclick=()=>stop('已停止 · 回到虚拟起点后可重新开启');
$('reset').onclick=()=>{stop();engine?.reset();};
$('preview').onclick=()=>{stop();engine.preview(engine.selected.id,performance.now());};
$('demo').onclick=()=>startDemo(false);
$('record').onclick=()=>startDemo(true);
for(const id of ['dx','dy','dz'])$(id).oninput=()=>{stop('正在设置模拟目标');engine.anchor=null;$(id+'Value').textContent=`${$(id).value} mm`;engine.setTarget(engine.lib.anchor_m.map((x,i)=>x+offsets()[i]));};
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop('页面隐藏 · 已停止');});
const watchdog=setInterval(()=>{if(engine?.enabled&&!demo&&(!latest||performance.now()-latest.atMs>500))stop('视觉超过 500 ms 未更新 · 已停止');},100);
window.addEventListener('pagehide',()=>{alive=false;stop();clearInterval(watchdog);cancelAnimationFrame(raf);closeCamera?.();scene?.close();});
(async()=>{
 try{
  const [data,library]=await Promise.all(['../reports/scooping/scene.json','feeding-library-100.json'].map(async url=>{const r=await fetch(url);if(!r.ok)throw Error(`加载失败 ${url}`);return r.json();}));
  engine=new SmartFeeder(library,createKinematics(data));scene=await createFeedingScene($('scene'),data);
  for(const plan of library.trajectories){const b=document.createElement('button');b.textContent=plan.id;b.dataset.id=plan.id;b.title=`终点 ${fmt(plan.virtual_mouth_m)} mm`;b.onclick=()=>{stop(`已选择 ${plan.id}，可手动播放`);engine.anchor=null;engine.selected=plan;};$('routeGrid').appendChild(b);}
  $('preview').disabled=false;$('demo').disabled=false;$('record').disabled=false;
  $('cameraConnect').onclick=()=>{stop('连接真实相机 · 等待稳定的嘴部深度');$('cameraConnect').disabled=true;$('offlineHint').hidden=true;$('visionStatus').textContent='正在加载本地人脸模型…';closeCamera=startMouthCamera({canvas:$('camera'),onSample:s=>{latest=s;$('cameraXYZ').textContent=s?fmt(s.xyz):'无有效三维位置';if(!demo)engine.ingest(s,performance.now());},onPixel:p=>{$('pixel').textContent=p?`${p.x.toFixed(1)} / ${p.y.toFixed(1)} · ${p.state}`:'—';},onStatus:(message,ok)=>{$('visionStatus').textContent=message;$('cameraTag').textContent=ok?'同帧 RGB-D':'等待视觉';}});};
  render(performance.now());
 }catch(error){$('runStatus').textContent=`初始化失败：${error.message}`;}
})();
