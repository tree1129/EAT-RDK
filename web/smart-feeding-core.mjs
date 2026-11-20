import {MouthGate,validateTrajectory,sampleTrajectory} from './mouth-gate.mjs';
import {MouthFollower} from './mouth-follow-core.mjs';
export const distance=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
const point=p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite);
export function validateLibrary(lib,kin){
 if(lib?.count!==100||lib.trajectories?.length!==100||lib.mode!=='simulation_only'||lib.hardware_execution_enabled!==false||lib.frame!=='base_link'||lib.units!=='radians_seconds'||!point(lib.anchor_m)||!Number.isFinite(lib.max_match_distance_m)||lib.max_match_distance_m<=0||lib.max_match_distance_m>.05||lib.gain!==.4||!Array.isArray(lib.home_q_rad)||lib.home_q_rad.length!==4||!lib.home_q_rad.every(Number.isFinite))throw Error('轨迹库格式无效');
 const ids=new Set(),ends=new Set();
 for(const plan of lib.trajectories){
  validateTrajectory(plan,lib.joint_names);
  if(!/^F\d{3}$/.test(plan.id)||ids.has(plan.id))throw Error('轨迹编号重复或无效');ids.add(plan.id);
  const end=kin.pose(plan.stages[1].samples.at(-1).q_rad);
  if(distance(end.point,plan.virtual_mouth_m)>.00001||end.tilt>.01)throw Error('送餐终点或姿态不匹配');
  ends.add(plan.virtual_mouth_m.map(x=>x.toFixed(7)).join(','));
  if(distance(plan.stages[0].samples[0].q_rad,lib.home_q_rad)>1e-6)throw Error('起点不一致');
  const duration=plan.stages.reduce((sum,s)=>sum+s.samples.at(-1).t_s,0);
  if(!Number.isFinite(plan.duration_s)||Math.abs(duration-plan.duration_s)>1e-5)throw Error('轨迹时长不匹配');
 }
 if(ends.size!==100)throw Error('送餐终点重复');return lib;
}
export function rankTrajectories(lib,target){
 if(!point(target))throw Error('目标坐标无效');
 return lib.trajectories.map(plan=>({plan,distance_m:distance(plan.virtual_mouth_m,target)})).sort((a,b)=>a.distance_m-b.distance_m||a.plan.id.localeCompare(b.plan.id));
}
export class SmartFeeder {
 constructor(lib,kin){this.lib=validateLibrary(lib,kin);this.kin=kin;this.vision=new MouthFollower(kin);this.gate=new MouthGate();this.q=[...lib.home_q_rad];this.target=[...lib.anchor_m];this.ranking=rankTrajectories(lib,this.target);this.selected=this.ranking[0].plan;this.anchor=null;this.enabled=false;this.run=null;this.elapsed=0;this.stage='idle';this.reason='待机 · 先设置嘴部基准';this.triggerCount=0;this.events=[];}
 event(type,detail={}){this.events.push({type,...detail});if(this.events.length>30)this.events.shift();}
 reset(){this.cancel('已回到虚拟起点');this.q=[...this.lib.home_q_rad];this.stage='idle';this.elapsed=0;}
 cancel(reason='已暂停'){this.run=null;this.enabled=false;this.gate.reset();this.reason=reason;this.event('stop',{reason});}
 enable(now){
  if(!this.vision.ready(now)||distance(this.q,this.lib.home_q_rad)>1e-6)return false;
  const h=this.vision.history;
  this.anchor=h[0].xyz.map((_,i)=>h.reduce((s,p)=>s+p.xyz[i],0)/h.length);
  this.source='live';this.enabled=true;this.gate.reset();this.reason='已开启 · 先闭嘴，再持续张嘴';return true;
 }
 enableDemo(){this.reset();this.vision=new MouthFollower(this.kin);this.anchor=[0,0,.65];this.source='demo';this.enabled=true;this.reason='模拟视觉输入 · 等待闭嘴再张嘴';}
 setTarget(target){this.target=[...target];this.ranking=rankTrajectories(this.lib,this.target);if(!this.run)this.selected=this.ranking[0].plan;}
 ingest(sample,now){
  const valid=this.vision.ingest(sample&&Number.isFinite(sample.ratio)&&sample.ratio>=0?sample:null,now);
  if(!valid){if(this.enabled||this.run?.source==='live')this.cancel('视觉无效或不连续 · 已暂停，需要重新设置基准');return false;}
  if(!this.anchor)return true;
  // Short average reduces depth noise; the selected route is frozen on trigger.
  const history=this.vision.history.filter(p=>sample.atMs-p.atMs<=350);
  const xyz=sample.xyz.map((_,i)=>history.reduce((n,p)=>n+p.xyz[i],0)/history.length);
  const target=this.lib.anchor_m.map((v,i)=>v+(xyz[i]-this.anchor[i])*this.lib.gain);
  this.setTarget(target);
  if(this.run){
   if(this.run.source!=='manual'){
    // Raw displacement also guards jumps that filtering could hide.
    const raw=this.lib.anchor_m.map((v,i)=>v+(sample.xyz[i]-this.anchor[i])*this.lib.gain);
    if(distance(raw,this.run.target)>.025)this.cancel('嘴部目标移动超过 25 mm · 已暂停');
    else if(['deliver','wait'].includes(this.stage)&&sample.ratio<=.08)this.cancel('送餐阶段检测到闭嘴 · 已暂停');
   }
   return true;
  }
  if(!this.enabled)return true;
  if(!Number.isFinite(sample.ratio)){this.cancel('嘴部开合数据无效');return false;}
  if(this.gate.update({...sample,nowMs:now})){
   const best=this.ranking[0];
   if(best.distance_m>this.lib.max_match_distance_m){this.reason='最近轨迹仍超过 25 mm · 本次不触发';this.event('no_match',{distance_m:best.distance_m});return true;}
   this.selected=best.plan;this.start(this.source,now);this.triggerCount++;this.event('selected',{id:best.plan.id,distance_mm:best.distance_m*1000});
  }else this.reason=this.gate.state;
  return true;
 }
 start(source,now){
  if(distance(this.q,this.lib.home_q_rad)>1e-6){this.cancel('请先回到虚拟起点');return false;}
  this.run={plan:this.selected,target:[...this.target],source,lastTick:now};this.elapsed=0;this.stage='scoop';this.gate.reset();this.reason=`播放 ${this.selected.id} · 取餐并调平`;return true;
 }
 preview(id,now){this.reset();this.anchor=null;this.selected=this.lib.trajectories.find(p=>p.id===id)||this.selected;this.setTarget(this.selected.virtual_mouth_m);return this.start('manual',now);}
 tick(now){
  if(!this.run){if(this.enabled&&(!this.vision.sample||now-this.vision.sample.atMs>500))this.cancel('视觉超过 500 ms 未更新 · 已暂停');return;}
  const run=this.run,dt=(now-run.lastTick)/1000;
  if(!Number.isFinite(dt)||dt<0||dt>.25){this.cancel('页面停顿 · 已暂停');return;}
  if(run.source!=='manual'&&(!this.vision.sample||now-this.vision.sample.atMs>500)){this.cancel('视觉超过 500 ms 未更新 · 已暂停');return;}
  const elapsed=Math.min(run.plan.duration_s,this.elapsed+dt),sample=sampleTrajectory(run.plan,elapsed);
  if(run.source!=='manual'&&['deliver','wait'].includes(sample.stage.id)&&this.vision.sample.ratio<=.08){this.cancel('送餐前需要保持张嘴 · 已暂停');return;}
  this.elapsed=elapsed;this.q=sample.q;this.stage=sample.stage.id;this.local=sample.local;run.lastTick=now;
  this.reason=`${run.source==='manual'?'手动仿真':'张嘴触发'} ${run.plan.id} · ${sample.stage.label}`;
  if(elapsed>=run.plan.duration_s){this.q=[...this.lib.home_q_rad];this.run=null;this.stage='idle';this.gate.reset();this.vision.history=[];this.reason='本轮完成 · 重新闭嘴再张嘴可触发下一轮';this.event('complete',{id:run.plan.id});}
 }
}
