// Pure simulation math and tracking state. No I/O or motor commands.
const norm=v=>Math.hypot(...v), sub=(a,b)=>a.map((v,i)=>v-b[i]);
const identity=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
function mul(a,b){const out=Array(16).fill(0);for(let r=0;r<4;r++)for(let c=0;c<4;c++)for(let k=0;k<4;k++)out[r*4+c]+=a[r*4+k]*b[k*4+c];return out;}
function rotation(axis,q){const [x,y,z]=axis.map(v=>v/norm(axis)),c=Math.cos(q),s=Math.sin(q),t=1-c;return [t*x*x+c,t*x*y-s*z,t*x*z+s*y,0,t*x*y+s*z,t*y*y+c,t*y*z-s*x,0,t*x*z-s*y,t*y*z+s*x,t*z*z+c,0,0,0,0,1];}
function linear(A,b){const m=A.map((r,i)=>[...r,b[i]]);for(let k=0;k<3;k++){let p=k;for(let j=k+1;j<3;j++)if(Math.abs(m[j][k])>Math.abs(m[p][k]))p=j;[m[k],m[p]]=[m[p],m[k]];if(Math.abs(m[k][k])<1e-12)return null;const d=m[k][k];for(let c=k;c<4;c++)m[k][c]/=d;for(let r=0;r<3;r++)if(r!==k){const f=m[r][k];for(let c=k;c<4;c++)m[r][c]-=f*m[k][c];}}return m.map(r=>r[3]);}
export function createKinematics(data){
 const {geometry,config,spoon}=data;
 const home=data.trajectories[data.selected.center].frames[0].q_rad;
 const offset=home[2]-home[1], expand=v=>[v[0],v[1],v[1]+offset,v[2]];
 const initial=expand([Math.PI/2,-1.25,-1.3]);
 const joints=geometry.joints.map(j=>({...j,origin:j.origin.flat(),index:config.joint_names.indexOf(j.name)}));
 const tcp=identity();[tcp[3],tcp[7],tcp[11]]=spoon.offset_m;
 function pose(q){
  const frames={[geometry.frame]:identity()},pending=new Set(joints);
  while(pending.size){let progress=false;for(const j of pending){if(!frames[j.parent])continue;let t=mul(frames[j.parent],j.origin);if(j.kind!=='fixed')t=mul(t,rotation(j.axis,q[j.index]));frames[j.child]=t;pending.delete(j);progress=true;}if(!progress)throw Error('URDF tree invalid');}
  const t=mul(frames[spoon.parent_link],tcp);
  return {point:[t[3],t[7],t[11]],tilt:Math.atan2(Math.hypot(t[1],t[9]),-t[5])*180/Math.PI};
 }
 function solve(target,seed){
  if(target.length!==3||!target.every(Number.isFinite))return null;
  let v=[seed[0],seed[1],seed[3]];
  // These are conservative demo branch bounds, NOT measured hardware limits.
  const lower=[.35,-1.85,-2.6],upper=[2.8,-.5,-.25];
  for(let iteration=0;iteration<32;iteration++){
   const q=expand(v),p=pose(q).point,e=sub(target,p);
   if(norm(e)<.00015)return q;
   const cols=v.map((_,i)=>{const w=[...v];w[i]+=1e-5;return sub(pose(expand(w)).point,p).map(x=>x/1e-5);});
   const A=cols.map((a,i)=>cols.map((b,j)=>a.reduce((s,x,k)=>s+x*b[k],0)+(i===j?1e-7:0)));
   const b=cols.map(a=>a.reduce((s,x,k)=>s+x*e[k],0)),d=linear(A,b);if(!d)return null;
   const scale=Math.min(1,.08/Math.max(...d.map(Math.abs)));
   v=v.map((x,i)=>x+d[i]*scale);
   if(v.some((x,i)=>x<lower[i]||x>upper[i]))return null;
  }
  return null;
 }
 return {pose,solve,initial,offset};
}

export class MouthFollower {
 constructor(kin){this.kin=kin;this.q=[...kin.initial];this.active=false;this.reason='等待稳定的单人嘴部深度';this.history=[];this.sample=null;this.lastId=null;this.gain=.4;this.target=kin.pose(this.q).point;this.filtered=[...this.target];}
 stop(reason='已暂停'){this.active=false;this.reason=reason;this.target=this.kin.pose(this.q).point;this.filtered=[...this.target];this.history=[];}
 reset(){this.q=[...this.kin.initial];this.stop('已回到虚拟起点');}
 ingest(s,now){
  if(!s||s.faces!==1||!s.frameId||!Array.isArray(s.xyz)||s.xyz.length!==3||!s.xyz.every(Number.isFinite)||!Number.isFinite(s.atMs)||now-s.atMs>500||now<s.atMs||s.xyz[2]<.2||s.xyz[2]>2){this.sample=null;this.stop('深度无效、丢脸或画面过期 · 请重新设置基准');return false;}
  if(s.frameId===this.lastId){this.sample=null;this.stop('重复相机帧 · 已暂停');return false;}
  if(this.sample){const dt=(s.atMs-this.sample.atMs)/1000,jump=norm(sub(s.xyz,this.sample.xyz));if(dt<=0||dt>.5||jump>.08||(dt>.02&&jump/dt>.8)){this.sample=null;this.stop('嘴部位置跳变或帧间隔过大 · 请重新设置基准');return false;}}
  this.lastId=s.frameId;this.sample=s;
  this.history.push(s);this.history=this.history.filter(x=>s.atMs-x.atMs<=1100);
  if(this.active){
   const delta=sub(s.xyz,this.anchor).map(x=>x*this.gain),bounds=[.05,.035,.035];
   if(delta.some((x,i)=>Math.abs(x)>bounds[i])){this.stop('超过虚拟跟随范围 · 请重新设置基准');return false;}
   this.target=this.origin.map((x,i)=>x+delta[i]);
  }
  return true;
 }
 ready(now){return this.sample&&now-this.sample.atMs<=350&&this.history.length>=4&&this.history.at(-1).atMs-this.history[0].atMs>=700&&this.history.every(s=>norm(sub(s.xyz,this.sample.xyz))<.018);}
 arm(now,gain=.4){
  if(!this.ready(now)||!Number.isFinite(gain)||gain<.1||gain>1)return false;
  this.gain=gain;this.anchor=this.sample.xyz.map((_,i)=>this.history.reduce((a,s)=>a+s.xyz[i],0)/this.history.length);
  this.origin=this.kin.pose(this.q).point;this.target=[...this.origin];this.filtered=[...this.origin];this.active=true;this.reason='正在跟随 · 勺面水平';return true;
 }
 tick(now,dt){
  if(!this.active)return;
  if(!this.sample||now-this.sample.atMs>500||!Number.isFinite(dt)||dt<=0||dt>.15){this.stop('视觉超时或页面停顿 · 已暂停');return;}
  const p=this.kin.pose(this.q).point,alpha=1-Math.exp(-dt/.18);
  this.filtered=this.filtered.map((x,i)=>x+(this.target[i]-x)*alpha);
  const d=sub(this.filtered,p),length=norm(d),s=Math.min(1,.03*dt/Math.max(length,1e-9));
  const next=this.kin.solve(p.map((x,i)=>x+d[i]*s),this.q);
  if(!next){this.stop('目标超出当前水平姿态的可达范围');return;}
  const delta=sub(next,this.q),rate=Math.min(1,(20*Math.PI/180)*dt/Math.max(...delta.map(Math.abs),1e-9));
  let candidate=this.q.map((q,i)=>q+delta[i]*rate);
  const moved=norm(sub(this.kin.pose(candidate).point,p));
  if(moved>.03*dt)candidate=this.q.map((q,i)=>q+(candidate[i]-q)*(.03*dt/moved));
  this.q=candidate;
 }
}
