import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {MouthGate,validateTrajectory,sampleTrajectory} from '../web/mouth-gate.mjs';
const plan=JSON.parse(readFileSync(new URL('../web/feeding-trajectory.json',import.meta.url)));
function feed(g,from,to,ratio,extra={}){let triggers=0;for(let t=from;t<=to;t+=100)triggers+=g.update({atMs:t,nowMs:t,frameId:String(t+1),ratio,...extra})?1:0;return triggers;}
test('requires closed then sustained open; held open never retriggers',()=>{
 const g=new MouthGate();assert.equal(feed(g,0,1500,.4),0);assert.equal(feed(g,1600,2100,.03),0);assert.equal(feed(g,2200,5000,.3),1);assert.equal(feed(g,5100,5600,.03),0);assert.equal(feed(g,5700,6500,.3),1);
});
test('short openings and threshold jitter do not count as sustained intent',()=>{
 const g=new MouthGate();feed(g,0,500,.03);feed(g,600,1000,.3);feed(g,1100,1200,.14);assert.equal(feed(g,1300,1900,.3),0);assert.equal(feed(g,2000,2000,.3),1);
});
test('missing face, multiple faces, invalid ratios and stale data reset closed requirement',()=>{
 for(const bad of [{faces:0},{faces:2},{ratio:NaN},{ratio:Infinity},{ratio:-1},{nowMs:2000},{frameId:null}]){
  const g=new MouthGate();feed(g,0,500,.02);feed(g,600,1100,.3);g.update({atMs:1200,nowMs:1200,frameId:'bad',ratio:.3,...bad});assert.equal(feed(g,1300,3000,.3),0);
 }
});
test('long gaps and backwards timestamps cannot complete a trigger',()=>{
 for(const time of [400,2000]){const g=new MouthGate();feed(g,0,500,.03);feed(g,600,900,.3);assert.equal(g.update({atMs:time,nowMs:time,frameId:'late',ratio:.3}),false);assert.equal(g.ready,false);}
});
test('duplicate proxy frames do not advance dwell',()=>{
 const g=new MouthGate();assert.equal(feed(g,0,4000,.03,{frameId:'same'}),0);assert.equal(g.ready,false);
});
test('config cannot invert thresholds or disable dwell',()=>{for(const bad of [{open:.05},{openMs:0},{close:NaN},{maxGapMs:-1}])assert.throws(()=>new MouthGate(bad));});
test('default trajectory validates and closes its loop',()=>{
 assert.equal(validateTrajectory(plan,plan.joint_names),plan);const total=plan.stages.reduce((n,s)=>n+s.samples.at(-1).t_s,0);assert.equal(sampleTrajectory(plan,plan.stages[0].samples.at(-1).t_s+1).stage.id,'deliver');assert.deepEqual(sampleTrajectory(plan,total).q,plan.stages[0].samples[0].q_rad);
});
test('rejects hardware mode, wrong units, wrong joints, missing stages and broken continuity',()=>{
 for(const change of [p=>p.hardware_execution_enabled=true,p=>p.mode='live',p=>p.units='motor_degrees',p=>p.joint_names.reverse(),p=>p.stages.pop(),p=>p.stages[1].samples[0].q_rad[0]+=.5,p=>p.stages[0].samples[1].t_s=0,p=>p.stages[0].samples[0].q_rad[0]=NaN,p=>p.stages[2].samples[1].q_rad[0]+=.01,p=>p.stages[3].samples.at(-1).q_rad[0]+=.01]){
  const copy=structuredClone(plan);change(copy);assert.throws(()=>validateTrajectory(copy,plan.joint_names));
 }
});
