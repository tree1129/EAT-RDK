import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createKinematics,MouthFollower} from '../web/mouth-follow-core.mjs';
const data=JSON.parse(readFileSync(new URL('../reports/scooping/scene.json',import.meta.url)));
const kin=createKinematics(data),dist=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
const sample=(t,xyz=[0,0,.65],extra={})=>({xyz,faces:1,frameId:`f-${t}`,atMs:t,...extra});
function armed(){const f=new MouthFollower(kin);for(let t=0;t<=900;t+=100)f.ingest(sample(t),t);assert.ok(f.arm(900));return f;}
test('forward kinematics matches independent Python URDF fixture',()=>{
 const pose=kin.pose(kin.initial);
 assert.ok(dist(pose.point,[-.0979266573058254,.2273556346748986,.25106185203971065])<1e-10);
 assert.ok(pose.tilt<1e-6);
});
test('horizontal inverse kinematics closes position targets across a 3D neighborhood',()=>{
 const origin=kin.pose(kin.initial).point;
 for(const x of [-.035,0,.035])for(const y of [-.025,0,.025])for(const z of [-.025,0,.025]){
  const target=origin.map((v,i)=>v+[x,y,z][i]),q=kin.solve(target,kin.initial);
  assert.ok(q,`IK missing ${x},${y},${z}`);const pose=kin.pose(q);
  assert.ok(dist(target,pose.point)<.00016);assert.ok(pose.tilt<1e-6);
 }
 assert.equal(kin.solve([10,10,10],kin.initial),null);
 assert.equal(kin.solve([NaN,0,0],kin.initial),null);
});
test('requires fresh stable dwell before arming',()=>{
 const f=new MouthFollower(kin);assert.equal(f.arm(0),false);
 for(let t=0;t<=500;t+=100)f.ingest(sample(t),t);assert.equal(f.arm(500),false);
 for(let t=600;t<=900;t+=100)f.ingest(sample(t),t);assert.ok(f.ready(900));assert.equal(f.arm(1500),false);
 assert.equal(f.arm(900,2),false);assert.ok(f.arm(900,.4));
});
test('invalid / multiple faces / depth / timestamps stop and never resume automatically',()=>{
 const cases=[null,sample(1000,undefined,{faces:2}),sample(1000,[0,0,0]),sample(1000,[0,0,NaN]),sample(1000,undefined,{atMs:0}),sample(1000,undefined,{atMs:1100}),sample(1000,undefined,{frameId:'f-900'})];
 for(const bad of cases){const f=armed(),before=[...f.q];f.ingest(bad,1000);assert.equal(f.active,false);f.ingest(sample(1100),1100);f.tick(1100,.05);assert.deepEqual(f.q,before);assert.equal(f.active,false);}
});
test('independent tick watchdog freezes without another camera callback',()=>{
 const f=armed();f.ingest(sample(1000,[.02,0,.65]),1000);f.tick(1000,.02);const q=[...f.q];f.tick(1600,.02);assert.equal(f.active,false);f.tick(1620,.02);assert.deepEqual(f.q,q);
});
test('frame stalls, position jumps, workspace exits, and pause discard target',()=>{
 for(const action of [f=>f.tick(1000,.3),f=>f.ingest(sample(1000,[.1,0,.65]),1000),f=>{for(let t=1000;t<=1800;t+=100)f.ingest(sample(t,[(t-900)/100*.016,0,.65]),t);},f=>f.stop()]){
  const f=armed();action(f);assert.equal(f.active,false);assert.deepEqual(f.target,kin.pose(f.q).point);
 }
});
test('3D continuous follow preserves level spoon, bounded speed and branch continuity',()=>{
 const f=armed();let maxTilt=0,maxSpeed=0,maxJoint=0,maxError=0,totalMovement=0;
 for(let t=916;t<=16900;t+=16){
  const u=(t-900)/1000,xyz=[.055*Math.sin(u*.5),.025*Math.sin(u*.7),.65+.025*Math.sin(u*.6)];
  if(t===916||Math.floor((t-916)/16)%6===0)f.ingest(sample(t,xyz),t);
  const before=[...f.q],p=kin.pose(before).point;f.tick(t,.016);assert.ok(f.active,f.reason);
  const pose=kin.pose(f.q),step=dist(pose.point,p);totalMovement+=step;
  maxTilt=Math.max(maxTilt,pose.tilt);maxSpeed=Math.max(maxSpeed,step/.016);
  maxJoint=Math.max(maxJoint,...f.q.map((q,i)=>Math.abs(q-before[i])/.016));
  maxError=Math.max(maxError,dist(pose.point,f.target));
 }
 assert.ok(totalMovement>.05);assert.ok(maxTilt<1e-6);assert.ok(maxSpeed<=.03001);assert.ok(maxJoint<=20*Math.PI/180+1e-7);assert.ok(maxError<.007);
 console.log(JSON.stringify({maxTiltDeg:maxTilt,maxSpeedMps:maxSpeed,maxJointDegps:maxJoint*180/Math.PI,maxTrackingErrorMm:maxError*1000,totalMovementM:totalMovement}));
});
test('browser modules have no robot command endpoints',()=>{
 for(const name of ['mouth-follow.js','mouth-follow-camera.mjs','mouth-follow-core.mjs']){
  const source=readFileSync(new URL(`../web/${name}`,import.meta.url),'utf8');
  assert.doesNotMatch(source,/8765|method\s*:\s*['"]POST|\/api\/(?:move|motor|command|execute)/);
 }
});
