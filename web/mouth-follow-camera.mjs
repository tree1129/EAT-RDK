import {mouthGeometry} from './face-geometry.mjs';
import {CAMERA_SOURCE as SOURCE} from './mouth-follow-config.mjs';
export function startMouthCamera({canvas,onSample,onStatus,onPixel}){
 const ctx=canvas.getContext('2d'),worker=new Worker(new URL('./feeding-face-worker.js',import.meta.url));
 let alive=true,ready=false,pending=null,id=0,lastId=null,timer;
 const bad=message=>{onStatus(message,false);onSample(null);};
 worker.onmessage=({data})=>{
  if(data.type==='ready'){ready=true;clearTimeout(timer);}
  if(data.type==='result'&&pending?.id===data.id){clearTimeout(pending.timer);pending.resolve(data);pending=null;}
  if(data.type==='error'){bad('人脸模型错误，请刷新页面');if(pending){clearTimeout(pending.timer);pending.resolve(null);pending=null;}}
 };
 worker.onerror=()=>bad('本地人脸模型无法加载');
 timer=setTimeout(()=>bad('本地模型加载超时'),30000);
 async function json(path){const r=await fetch(SOURCE+path,{cache:'no-store',signal:AbortSignal.timeout(1200)});const j=await r.json();if(!r.ok||!j.ok)throw Error(j.error||'RGB-D 接口不可用');return j;}
 async function loop(){
  if(!alive)return;
  try{
   if(document.hidden||!ready){onSample(null);return;}
   const start=performance.now(),frame=await json('/api/rgbd/frame');
   if(!frame.frame_id||frame.frame_id===lastId||!Number.isFinite(frame.age_ms)||frame.age_ms<0||frame.age_ms>400||frame.aligned_to!=='color')throw Error('相机帧重复、过期或未对齐');
   lastId=frame.frame_id;
   const blob=await (await fetch('data:image/jpeg;base64,'+frame.rgb_jpeg_base64)).blob();
   const bitmap=await createImageBitmap(blob);
   if(!alive){bitmap.close();return;}
   canvas.width=bitmap.width;canvas.height=bitmap.height;ctx.drawImage(bitmap,0,0);bitmap.close();
   const input=await createImageBitmap(canvas),requestId=++id;
   const result=await new Promise(resolve=>{pending={id:requestId,resolve,timer:setTimeout(()=>{pending=null;resolve(null);},1200)};worker.postMessage({id:requestId,bitmap:input,timestamp:performance.now()},[input]);});
   if(!alive)return;
   if(!result||document.hidden||performance.now()-start+frame.age_ms>500)throw Error('识别延迟过大');
   const mouth=mouthGeometry(result.points,canvas.width,canvas.height);
   if(result.faceCount!==1||!mouth){onPixel(null);bad(result.faceCount>1?'检测到多人 · 跟随已暂停':'未检测到清晰人脸 · 等待单人入镜');return;}
   const lip=[61,40,37,0,267,270,291,321,314,17,84,91,61];
   ctx.strokeStyle='#96f0cc';ctx.lineWidth=2;ctx.beginPath();lip.forEach((n,i)=>{const p=result.points[n];ctx[i?'lineTo':'moveTo'](p.x*canvas.width,p.y*canvas.height);});ctx.stroke();
   const {x,y}=mouth.center;ctx.strokeStyle='#ffbf88';ctx.beginPath();ctx.moveTo(x-13,y);ctx.lineTo(x+13,y);ctx.moveTo(x,y-13);ctx.lineTo(x,y+13);ctx.stroke();
   onPixel({x,y,ratio:mouth.ratio,state:mouth.state});
   const pointStart=performance.now();
   const point=await json(`/api/rgbd/point?frame_id=${encodeURIComponent(frame.frame_id)}&u=${x.toFixed(3)}&v=${y.toFixed(3)}`);
   const atMs=Math.min(start-frame.age_ms,pointStart-point.age_ms);
   if(point.frame_id!==frame.frame_id||point.coordinate_frame!=='color_camera_optical'||!Number.isFinite(point.age_ms)||point.age_ms<0||performance.now()-atMs>500||document.hidden)throw Error('深度帧不匹配或已过期');
   onStatus(`同帧 RGB-D · ${point.valid_pixels}/49 有效深度 · 嘴部${mouth.state}`,true);
   onSample({xyz:point.xyz_m,pixel:[x,y],ratio:mouth.ratio,faces:1,frameId:frame.frame_id,atMs,ageMs:performance.now()-atMs,spread:point.spread_m});
  }catch(error){if(alive)bad(`等待可靠深度：${error.message}`);}
  finally{if(alive)setTimeout(loop,100);}
 }
 loop();
 return ()=>{alive=false;clearTimeout(timer);worker.terminate();if(pending){clearTimeout(pending.timer);pending.resolve(null);pending=null;}};
}
