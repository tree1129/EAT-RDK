import {createFeedingScene} from './feeding-scene.js';
import {createKinematics,MouthFollower} from './mouth-follow-core.mjs';
import {startMouthCamera} from './mouth-follow-camera.mjs';
const $=id=>document.getElementById(id),fmt=p=>p.map(x=>(x*1000).toFixed(1)).join(' / ');
let follower,scene,closeCamera,latest=null,demo=null,lastTime=performance.now(),raf,alive=true;
function pause(message){demo=null;if(follower)follower.stop(message);$('demoStatus').textContent='';}
function status(text,ok){$('visionStatus').textContent=text;$('cameraTag').textContent=ok?'同帧 RGB-D':'等待视觉';}
function ingest(s){latest=s;if(s)$('cameraXYZ').textContent=fmt(s.xyz);else $('cameraXYZ').textContent='无有效三维位置';if(!demo)follower.ingest(s,performance.now());}
function render(now){
 if(!alive)return;raf=requestAnimationFrame(render);if(!follower)return;
 const dt=(now-lastTime)/1000;lastTime=now;
 if(demo){
  const t=(now-demo.start)/1000;
  if(t>18){pause('离线验证完成 · 等待重新设置实时基准');}
  else {
   const u=Math.max(0,t-1.2),a=Math.min(1,u/2),point=[a*.06*Math.sin(u*.5),a*.035*Math.sin(u*.7),.65+a*.03*Math.sin(u*.6)];
   follower.ingest({xyz:point,faces:1,frameId:`demo-${now}`,atMs:now},now);
   if(t>.9&&!demo.armed)demo.armed=follower.arm(now,.4);
   $('demoStatus').textContent=`模拟输入 ${t.toFixed(1)} / 18 s`;
  }
 }
 follower.tick(now,dt);
 scene.setMouth(follower.target);scene.draw(follower.q,'scoop',0,false);
 const pose=follower.kin.pose(follower.q),error=Math.hypot(...pose.point.map((x,i)=>x-follower.target[i]));
 $('tilt').textContent=`${pose.tilt.toFixed(3)}°`;$('error').textContent=`${(error*1000).toFixed(1)} mm`;
 const age=latest?now-latest.atMs:null;$('age').textContent=age===null?'—':age>9999?'> 10 s':`${Math.round(age)} ms`;
 $('targetXYZ').textContent=fmt(follower.target);$('runStatus').textContent=!demo&&!follower.active&&follower.ready(now)?'嘴部三维定位稳定 · 可以设置基准并开始跟随':follower.reason;
 $('mode').textContent=demo?'离线模拟输入':follower.active?'实时相对跟随':'已暂停';
 $('arm').disabled=!!demo||follower.active||!follower.ready(now);
 $('gain').disabled=!!demo;
}
$('arm').onclick=()=>{if(follower.arm(performance.now(),Number($('gain').value)))$('demoStatus').textContent='';};
$('stop').onclick=()=>pause('已暂停 · 保持当前虚拟姿态');
$('reset').onclick=()=>{pause();follower?.reset();};
$('gain').oninput=()=>{pause('跟随比例已改变 · 请重新设置嘴部基准');$('gainValue').textContent=`${Number($('gain').value).toFixed(1)}×`;};
$('demo').onclick=()=>{pause();follower.reset();follower.sample=null;follower.lastId=null;demo={start:performance.now(),armed:false};};
document.addEventListener('visibilitychange',()=>{if(document.hidden)pause('页面已隐藏 · 跟随暂停');});
// Independent watchdog: loss of camera callbacks must stop motion too.
const watchdog=setInterval(()=>{if(follower?.active&&!demo&&(!latest||performance.now()-latest.atMs>500))pause('视觉数据超过 500 ms · 跟随暂停');},100);
window.addEventListener('pagehide',()=>{alive=false;pause();clearInterval(watchdog);cancelAnimationFrame(raf);closeCamera?.();scene?.close();});
(async()=>{
 try{
  const response=await fetch('../reports/scooping/scene.json');if(!response.ok)throw Error('模型数据不可用');
  const data=await response.json();follower=new MouthFollower(createKinematics(data));scene=await createFeedingScene($('scene'),data);
  $('demo').disabled=false;
  closeCamera=startMouthCamera({canvas:$('camera'),onSample:ingest,onStatus:status,onPixel:p=>{$('pixel').textContent=p?`${p.x.toFixed(1)} / ${p.y.toFixed(1)} · ${p.state}`:'—';}});
  lastTime=performance.now();render(lastTime);
 }catch(e){$('runStatus').textContent=`加载失败：${e.message}`;}
})();
