import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createKinematics} from '../web/mouth-follow-core.mjs';
import {SmartFeeder,rankTrajectories,validateLibrary,distance} from '../web/smart-feeding-core.mjs';
const read=p=>JSON.parse(readFileSync(new URL(p,import.meta.url)));
const lib=read('../web/feeding-library-100.json'),kin=createKinematics(read('../reports/scooping/scene.json'));
const sample=(t,ratio=.03,xyz=[0,0,.65],extra={})=>({faces:1,atMs:t,frameId:`frame-${t}`,ratio,xyz,...extra});
function ready(){const f=new SmartFeeder(lib,kin);for(let t=0;t<=900;t+=100)f.ingest(sample(t),t);assert.ok(f.enable(900));return f;}
function trigger(f){for(let t=1000;t<=1500;t+=100)f.ingest(sample(t),t);for(let t=1600;t<=2300;t+=100)f.ingest(sample(t,.28),t);assert.ok(f.run);return 2300;}
test('100 complete distinct reachable endpoints with horizontal delivery and a common home',()=>{
 assert.equal(validateLibrary(lib,kin),lib);
 assert.equal(new Set(lib.trajectories.map(p=>p.id)).size,100);
 for(const p of lib.trajectories){assert.equal(p.stages.length,4);assert.ok(p.validation.horizontal_max_tilt_deg<.01);assert.ok(p.validation.transport_max_speed_m_s<=.0301);assert.ok(p.validation.transport_max_joint_speed_deg_s<=20.01);assert.equal(p.validation.collision_checked,false);}
});
test('nearest selector returns exact endpoint IDs and matches independent brute-force distance',()=>{
 for(const p of lib.trajectories)assert.equal(rankTrajectories(lib,p.virtual_mouth_m)[0].plan.id,p.id);
 for(let i=0;i<150;i++){const target=lib.anchor_m.map((x,j)=>x+Math.sin(i*1.7+j)*.07),rank=rankTrajectories(lib,target),brute=lib.trajectories.map(p=>({id:p.id,d:p.virtual_mouth_m.reduce((s,x,j)=>s+(x-target[j])**2,0)})).sort((a,b)=>a.d-b.d||a.id.localeCompare(b.id));assert.equal(rank[0].plan.id,brute[0].id);assert.ok(Math.abs(rank[0].distance_m-Math.sqrt(brute[0].d))<1e-12);}
 assert.throws(()=>rankTrajectories(lib,[NaN,0,0]));
});
test('rejects malformed counts, endpoints, hardware mode and durations',()=>{
 for(const mutate of [v=>v.count=99,v=>v.hardware_execution_enabled=true,v=>v.trajectories[0].virtual_mouth_m[0]+=.1,v=>v.trajectories[0].duration_s=NaN]){const v=structuredClone(lib);mutate(v);assert.throws(()=>validateLibrary(v,kin));}
});
test('closed then sustained open triggers the nearest route, which stays locked while running',()=>{
 const f=ready(),expected=rankTrajectories(lib,lib.anchor_m)[0].plan.id;trigger(f);assert.equal(f.run.plan.id,expected);assert.equal(f.triggerCount,1);
 f.ingest(sample(2400,.28,[.02,0,.65]),2400);assert.equal(f.run.plan.id,expected);
 for(let t=2400;t<4000;t+=100){f.ingest(sample(t+100,.28,[.02,0,.65]),t+100);f.tick(t+100);}assert.equal(f.triggerCount,1);
});
test('mapped real-style camera displacement selects the corresponding distinct endpoint',()=>{
 const f=ready(),target=lib.trajectories[36].virtual_mouth_m;
 const xyz=target.map((v,i)=>(v-lib.anchor_m[i])/.4+[0,0,.65][i]);
 for(let t=1000;t<=3000;t+=100){const u=Math.min(1,(t-1000)/1000);f.ingest(sample(t,.03,xyz.map((v,i)=>[0,0,.65][i]+(v-[0,0,.65][i])*u)),t);}
 for(let t=3100;t<=3900;t+=100)f.ingest(sample(t,.28,xyz),t);
 assert.ok(f.run,f.reason);assert.equal(f.run.plan.id,'F037');assert.ok(distance(f.target,target)<1e-8);
});
test('excessive nearest distance refuses automatic playback',()=>{
 const f=new SmartFeeder(lib,kin);f.enableDemo();
 for(let t=0;t<=600;t+=100)f.ingest(sample(t,.03,[.6,0,.65]),t);
 for(let t=700;t<=1600;t+=100)f.ingest(sample(t,.28,[.6,0,.65]),t);
 assert.equal(f.run,null);assert.equal(f.triggerCount,0);assert.ok(f.events.some(e=>e.type==='no_match'));
});
test('lost / stale / multiple / duplicate frames and moved target stop without rewinding',()=>{
 for(const s of [null,sample(2400,NaN),sample(2400,-.1),sample(2400,.28,undefined,{faces:2}),sample(2400,.28,undefined,{frameId:'frame-2300'}),sample(0,.28),sample(2400,.28,[.075,0,.65])]){
  const f=ready();trigger(f);f.tick(2350);const before=[...f.q];f.ingest(s,2400);assert.equal(f.run,null);f.tick(2450);assert.deepEqual(f.q,before);assert.equal(f.enabled,false);
 }
});
test('independent callback timeout and long browser pause freeze playback',()=>{
 for(const t of [2900,4000]){const f=ready();trigger(f);f.tick(2350);const before=[...f.q];f.tick(t);assert.equal(f.run,null);assert.deepEqual(f.q,before);}
});
test('closed mouth at delivery transition stops; manual preview is independent of missing camera',()=>{
 const f=ready();let t=trigger(f);const stopAt=f.run.plan.stages[0].samples.at(-1).t_s;
 while(f.run&&f.elapsed<stopAt+.2){t+=100;f.ingest(sample(t,.03),t);f.tick(t);}
 assert.equal(f.run,null);assert.match(f.reason,/张嘴/);
 f.preview('F100',t);f.ingest(null,t+50);f.tick(t+100);assert.ok(f.run);assert.equal(f.run.plan.id,'F100');
});
test('full cycle returns home; held open never repeats, close/open permits the next cycle',()=>{
 const f=ready();let t=trigger(f);
 while(f.run){t+=100;f.ingest(sample(t,.28),t);f.tick(t);assert.ok(t<100000);}
 assert.deepEqual(f.q,lib.home_q_rad);assert.equal(f.triggerCount,1);
 for(let i=0;i<20;i++){t+=100;f.ingest(sample(t,.28),t);f.tick(t);}assert.equal(f.triggerCount,1);
 for(let i=0;i<6;i++){t+=100;f.ingest(sample(t,.03),t);}
 for(let i=0;i<9;i++){t+=100;f.ingest(sample(t,.28),t);}assert.equal(f.triggerCount,2);
});
