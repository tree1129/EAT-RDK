import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {STLLoader} from './vendor/STLLoader.js';
export async function createFeedingScene(container,data){
 const {config,geometry,spoon,particles}=data;
  const scene=new THREE.Scene(),root=new THREE.Group();root.rotation.x=Math.PI;scene.add(root);
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);
  const camera=new THREE.PerspectiveCamera(37,1,.001,10),controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
  scene.add(new THREE.HemisphereLight(0xecf8ff,0x5d6261,2.5));
  const light=new THREE.DirectionalLight(0xfff2d6,3);light.position.set(-.4,1,-.2);scene.add(light);
  const fill=new THREE.DirectionalLight(0x95b5db,1.6);fill.position.set(.6,0,-.5);scene.add(fill);
  const matrix=rows=>new THREE.Matrix4().set(...rows.flat()),origins=geometry.joints.map(j=>matrix(j.origin));
  const axes=geometry.joints.map(j=>new THREE.Vector3(...j.axis).normalize());
  function forward(q){
    const result={[geometry.frame]:new THREE.Matrix4()},pending=new Set(geometry.joints.map((_,i)=>i));
    while(pending.size){let advance=false;for(const i of pending){const j=geometry.joints[i];if(!result[j.parent])continue;const t=result[j.parent].clone().multiply(origins[i]);if(j.kind!=='fixed')t.multiply(new THREE.Matrix4().makeRotationAxis(axes[i],q[config.joint_names.indexOf(j.name)]));result[j.child]=t;pending.delete(i);advance=true;}if(!advance)throw Error('URDF 关系无法解析');}return result;
  }
  const meshes=[],loader=new STLLoader();
  await Promise.all(geometry.visuals.map(async v=>{
    const g=await loader.loadAsync(`../models/${v.mesh}`),m=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:v.link==='chassis'?0x718c91:new THREE.Color(...v.color.slice(0,3)),metalness:.2,roughness:.55}));
    m.matrixAutoUpdate=false;root.add(m);meshes.push({mesh:m,link:v.link,origin:matrix(v.origin),scale:new THREE.Matrix4().makeScale(...v.scale)});
  }));
  // Assumed spoon: curved open bowl, visible rim, and handle attached to the CAD mount.
  const spoonGroup=new THREE.Group();spoonGroup.matrixAutoUpdate=false;root.add(spoonGroup);
  const vertices=[],indices=[],rings=10,sectors=48;
  for(let r=0;r<=rings;r++)for(let a=0;a<=sectors;a++){const rho=r/rings,theta=a/sectors*Math.PI*2;vertices.push(spoon.half_length_m*rho*Math.cos(theta),-spoon.depth_m*(1-rho*rho),spoon.half_width_m*rho*Math.sin(theta));}
  for(let r=0;r<rings;r++)for(let a=0;a<sectors;a++){const i=r*(sectors+1)+a,j=i+sectors+1;indices.push(i,j,i+1,j,j+1,i+1);}
  const bowlGeometry=new THREE.BufferGeometry();bowlGeometry.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));bowlGeometry.setIndex(indices);bowlGeometry.computeVertexNormals();
  spoonGroup.add(new THREE.Mesh(bowlGeometry,new THREE.MeshStandardMaterial({color:0xc2e3e7,metalness:.65,roughness:.24,side:THREE.DoubleSide})));
  const rimPoints=Array.from({length:65},(_,i)=>new THREE.Vector3(spoon.half_length_m*Math.cos(i/64*2*Math.PI),0,spoon.half_width_m*Math.sin(i/64*2*Math.PI)));
  const rim=new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(rimPoints),64,.0008,6,false),new THREE.MeshStandardMaterial({color:0x67d8e6,metalness:.4,roughness:.3}));spoonGroup.add(rim);
  const handle=new THREE.Mesh(new THREE.BoxGeometry(.055,.002,.006),new THREE.MeshStandardMaterial({color:0x4fbacb,metalness:.4,roughness:.3}));handle.position.x=-.043;spoonGroup.add(handle);
  const food=new THREE.InstancedMesh(new THREE.SphereGeometry(1,8,6),new THREE.MeshStandardMaterial({color:0xf1e5bd,roughness:.94}),particles.length);food.instanceMatrix.setUsage(THREE.DynamicDrawUsage);food.frustumCulled=false;root.add(food);
  const grain=new THREE.Object3D(),grainMatrix=new THREE.Matrix4();
  particles.forEach((p,i)=>food.setColorAt(i,new THREE.Color().setHSL(.115+(i%7)*.002,.3,.74+(i%11)*.014)));

 const marker=new THREE.Mesh(new THREE.TorusGeometry(.023,.002,10,48),new THREE.MeshStandardMaterial({color:0xf0a4b3,emissive:0x502331}));root.add(marker);
 const libraryGroup=new THREE.Group();root.add(libraryGroup);
 const routeLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x8ee0cc,transparent:true,opacity:.65}));root.add(routeLine);
 const foodBall=new THREE.Mesh(new THREE.SphereGeometry(.009,16,10),new THREE.MeshStandardMaterial({color:0xeee3bf,roughness:1}));foodBall.scale.set(1,.6,.8);spoonGroup.add(foodBall);foodBall.visible=false;
 const grid=new THREE.GridHelper(1.5,40,0x486269,0x2d454c);grid.position.y=-.425;scene.add(grid);
 controls.target.set(.02,-.20,-.12);camera.position.set(-.65,.22,-.88);controls.update();
 let raf,alive=true,recording=null,lastTilt=0;
 function paintRecording(){
  if(!recording)return;
  const r=recording;r.renderer.render(scene,r.camera);
  r.ctx.fillStyle='#15262b';r.ctx.fillRect(0,0,1280,720);r.ctx.drawImage(r.renderer.domElement,0,0);
  r.ctx.fillStyle='#102025ee';r.ctx.fillRect(0,0,1280,66);r.ctx.fillRect(0,652,1280,68);
  r.ctx.font='24px sans-serif';r.ctx.fillStyle='#e7eee8';r.ctx.fillText('EAT-RDK · 视觉交互喂饭仿真',32,41);
  r.ctx.font='18px sans-serif';r.ctx.fillStyle='#c2d8a9';r.ctx.fillText('仅仿真 · 未驱动实机',1030,40);
  r.ctx.fillStyle='#e7eee8';r.ctx.font='22px sans-serif';r.ctx.fillText(r.label||'准备',32,694);
  r.ctx.font='18px sans-serif';r.ctx.fillText(`勺面倾角 ${lastTilt.toFixed(3)}°`,670,694);
  r.ctx.fillText(`${(r.time||0).toFixed(1)} / ${(r.total||0).toFixed(1)} s`,1080,694);
 }

 const observer=new ResizeObserver(()=>{const r=container.getBoundingClientRect();renderer.setSize(r.width,r.height);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();});observer.observe(container);
 function animate(){if(!alive)return;raf=requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);paintRecording();}animate();
 return {
  beginRecording(){
   if(recording)throw Error('已经在录制');
   if(typeof MediaRecorder==='undefined'||!HTMLCanvasElement.prototype.captureStream)throw Error('当前浏览器不支持画布录制');
   const type=['video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm','video/mp4'].find(t=>MediaRecorder.isTypeSupported(t));
   if(!type)throw Error('没有可用的视频编码器');
   const output=document.createElement('canvas');output.width=1280;output.height=720;
   const rr=new THREE.WebGLRenderer({antialias:true,alpha:true,preserveDrawingBuffer:true});rr.setSize(1280,720);rr.setPixelRatio(1);rr.outputColorSpace=THREE.SRGBColorSpace;
   const cam=camera.clone();cam.aspect=1280/720;cam.updateProjectionMatrix();
   const stream=output.captureStream(30),recorder=new MediaRecorder(stream,{mimeType:type,videoBitsPerSecond:6000000}),chunks=[];
   const done=new Promise((resolve,reject)=>{recorder.ondataavailable=e=>{if(e.data.size)chunks.push(e.data);};recorder.onstop=()=>resolve(new Blob(chunks,{type:recorder.mimeType}));recorder.onerror=e=>reject(Error(e.error?.message||'视频编码失败'));});
   recording={renderer:rr,camera:cam,ctx:output.getContext('2d'),recorder,stream,done};controls.enabled=false;paintRecording();recorder.start(1000);
  },
  recordingFrame(label,time,total){if(recording)Object.assign(recording,{label,time,total});},
  async finishRecording(){
   if(!recording)return null;paintRecording();
   const r=recording;recording=null;controls.enabled=true;
   try{r.recorder.stop();return await r.done;}finally{r.stream.getTracks().forEach(t=>t.stop());r.renderer.dispose();}
  },
  tilt(){return lastTilt;},
  setMouth(point){marker.position.fromArray(point);},
  setLibraryTargets(points,selected=-1){
   if(libraryGroup.children.length!==points.length){
    for(const child of [...libraryGroup.children]){libraryGroup.remove(child);child.geometry.dispose();child.material.dispose();}
    points.forEach(p=>{const m=new THREE.Mesh(new THREE.SphereGeometry(.0018,8,6),new THREE.MeshBasicMaterial({color:0x63898e}));m.position.fromArray(p);libraryGroup.add(m);});
   }
   libraryGroup.children.forEach((m,i)=>{m.material.color.setHex(i===selected?0xffc389:0x63898e);m.scale.setScalar(i===selected?2.4:1);});
  },
  setTrajectory(points){routeLine.geometry.dispose();routeLine.geometry=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p)));},
  draw(q,stage,local,defaultScoop){
   const transforms=forward(q);
   for(const m of meshes)m.mesh.matrix.copy(transforms[m.link]).multiply(m.origin).multiply(m.scale);
   spoonGroup.matrix.copy(transforms[spoon.parent_link]).multiply(new THREE.Matrix4().makeTranslation(...spoon.offset_m));
   lastTilt=THREE.MathUtils.radToDeg(Math.acos(THREE.MathUtils.clamp(-spoonGroup.matrix.elements[5],-1,1)));
   const track=data.trajectories[data.selected.center];
   const index=defaultScoop?Math.min(track.frames.length-1,Math.floor(local/.05)):-1;
   const attached=new Map(defaultScoop?track.events.filter(e=>stage!=='scoop'||e.frame<=index).map(e=>[e.particle_id,e]):[]);
   for(const p of particles){const e=attached.get(p.id);grain.position.fromArray(e?e.local_m:p.position_m);grain.rotation.set(0,p.yaw_rad,0);grain.scale.set(.003*p.scale,.00125*p.scale,.00145*p.scale);if(e&&stage==='retreat')grain.scale.set(0,0,0);grain.updateMatrix();grainMatrix.copy(e?spoonGroup.matrix:new THREE.Matrix4()).multiply(grain.matrix);food.setMatrixAt(p.id,grainMatrix);}food.instanceMatrix.needsUpdate=true;
   foodBall.visible=!defaultScoop&&['deliver','wait'].includes(stage);
  },
  close(){alive=false;cancelAnimationFrame(raf);observer.disconnect();controls.dispose();scene.traverse(o=>{o.geometry?.dispose();if(Array.isArray(o.material))o.material.forEach(m=>m.dispose());else o.material?.dispose();});renderer.dispose();}
 };
}
