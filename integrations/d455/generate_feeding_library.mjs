// Generate complete simulation trajectories from the existing CAD/FK and scoop.
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {createKinematics} from '../../web/mouth-follow-core.mjs';
import {validateTrajectory} from '../../web/mouth-gate.mjs';
const root=new URL('../../',import.meta.url);
const raw=readFileSync(new URL('reports/scooping/scene.json',root));
const data=JSON.parse(raw),seed=JSON.parse(readFileSync(new URL('web/feeding-trajectory.json',root))),kin=createKinematics(data);
const home=seed.stages[0].samples[0].q_rad,level=seed.stages[0].samples.at(-1).q_rad,arrival=seed.stages[2].samples[0].q_rad;
const dist=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
function segment(a,b,minimum=2){
 let seconds=Math.max(minimum,1.875*Math.max(...a.map((x,i)=>Math.abs(x-b[i])))/(20*Math.PI/180));
 const points=Array.from({length:101},(_,i)=>{const u=i/100,v=10*u**3-15*u**4+6*u**5;return kin.pose(a.map((x,j)=>x+(b[j]-x)*v)).point;});
 const peak=Math.max(...points.slice(1).map((p,i)=>dist(p,points[i])*100));
 seconds=Math.ceil(Math.max(seconds,peak/.03)*20)/20+.05;
 const n=Math.ceil(seconds/.05);
 return Array.from({length:n+1},(_,i)=>{const u=i/n,v=10*u**3-15*u**4+6*u**5;return {t_s:Math.round(seconds*u*1e6)/1e6,q_rad:a.map((x,j)=>x+(b[j]-x)*v)};});
}
const trajectories=[],summaries=[];
for(const yaw of [-.16,-.08,0,.08,.16])for(const pitch of [-.18,-.09,0,.09,.18])for(const wrist of [-.24,-.08,.08,.24]){
 const q=arrival.map((x,i)=>x+[yaw,pitch,pitch,wrist][i]),point=kin.pose(q).point;
 const back=segment(q,level),finish=segment(level,home);
 const plan={schema:1,id:`F${String(trajectories.length+1).padStart(3,'0')}`,name:'水平送餐仿真',mode:'simulation_only',hardware_execution_enabled:false,
  frame:'base_link',units:'radians_seconds',joint_names:seed.joint_names,virtual_mouth_m:point,
  stages:[structuredClone(seed.stages[0]),{id:'deliver',label:'水平送餐',samples:segment(level,q)},
   {id:'wait',label:'嘴前等待（仿真）',samples:[{t_s:0,q_rad:q},{t_s:2,q_rad:q}]},
   {id:'retreat',label:'水平撤回',samples:[...back,...finish.slice(1).map(s=>({...s,t_s:s.t_s+back.at(-1).t_s}))]}]};
 validateTrajectory(plan,seed.joint_names);
 let tilt=0,speed=0,jointSpeed=0,maxStep=0,scoopTilt=0;
 for(const stage of plan.stages){
  const samples=stage.samples;
  for(let i=1;i<samples.length;i++){
   const a=samples[i-1],b=samples[i],dt=b.t_s-a.t_s;
   maxStep=Math.max(maxStep,...a.q_rad.map((v,j)=>Math.abs(v-b.q_rad[j])));
   for(const u of [0,.5,1]){
    const q=a.q_rad.map((v,j)=>v+(b.q_rad[j]-v)*u),pose=kin.pose(q);
    if(stage.id==='scoop')scoopTilt=Math.max(scoopTilt,pose.tilt);else tilt=Math.max(tilt,pose.tilt);
   }
   if(stage.id!=='scoop'){
    speed=Math.max(speed,dist(kin.pose(a.q_rad).point,kin.pose(b.q_rad).point)/dt);
    jointSpeed=Math.max(jointSpeed,...a.q_rad.map((v,j)=>Math.abs(v-b.q_rad[j])/dt));
   }
  }
 }
 if(tilt>.01||speed>.0301||jointSpeed>20.01*Math.PI/180)throw Error(`trajectory bounds failed ${plan.id}`);
 plan.duration_s=plan.stages.reduce((n,s)=>n+s.samples.at(-1).t_s,0);
 plan.validation={horizontal_max_tilt_deg:tilt,transport_max_speed_m_s:speed,transport_max_joint_speed_deg_s:jointSpeed*180/Math.PI,
  scoop_max_tilt_deg:scoopTilt,max_sample_step_rad:maxStep,collision_checked:false,physical_joint_limits_verified:false};
 summaries.push({id:plan.id,endpoint_m:point,duration_s:plan.duration_s,...plan.validation});trajectories.push(plan);
}
if(new Set(trajectories.map(p=>p.virtual_mouth_m.map(v=>v.toFixed(7)).join(','))).size!==100)throw Error('duplicate endpoints');
const library={schema:1,mode:'simulation_only',hardware_execution_enabled:false,frame:'base_link',units:'radians_seconds',
 joint_names:seed.joint_names,count:100,selection_metric:'euclidean_distance_to_delivery_endpoint',max_match_distance_m:.025,
 anchor_m:seed.virtual_mouth_m,home_q_rad:home,scoop_source:'reports/scooping/scene.json',source_sha256:createHash('sha256').update(raw).digest('hex'),
 mapping:'relative_camera_displacement_only_not_calibrated',gain:.4,
 note:'100 条完整几何/运动学仿真轨迹。共用已有取餐段；送餐终点不同。未验证全机碰撞、实物接触或硬件限位。',trajectories};
writeFileSync(new URL('web/feeding-library-100.json',root),JSON.stringify(library)+'\n');
writeFileSync(new URL('reports/feeding-library-100-validation.json',root),JSON.stringify({count:100,checks:'all samples and midpoints',trajectories:summaries},null,2)+'\n');
console.log(JSON.stringify({count:100,duration_s:[Math.min(...summaries.map(p=>p.duration_s)),Math.max(...summaries.map(p=>p.duration_s))],
 max_horizontal_tilt_deg:Math.max(...summaries.map(p=>p.horizontal_max_tilt_deg)),max_transport_speed_m_s:Math.max(...summaries.map(p=>p.transport_max_speed_m_s)),
 max_transport_joint_speed_deg_s:Math.max(...summaries.map(p=>p.transport_max_joint_speed_deg_s))}));
