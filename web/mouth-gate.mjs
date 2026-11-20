// Intent only. This module has no transport, hardware client or motion API.
export class MouthGate {
  constructor({open=.18, close=.08, openMs=700, closeMs=400, maxGapMs=500}={}) {
    if(![open,close,openMs,closeMs,maxGapMs].every(Number.isFinite)||close<0||open<=close||openMs<=0||closeMs<=0||maxGapMs<=0)throw Error('识别参数无效');
    Object.assign(this,{open,close,openMs,closeMs,maxGapMs});this.reset();
  }
  reset(){this.ready=false;this.since=null;this.lastTime=null;this.lastId=null;this.state='等待闭嘴';}
  update({ratio,faces=1,frameId,atMs,nowMs=atMs}) {
    if(faces!==1||!Number.isFinite(ratio)||ratio<0||!Number.isFinite(atMs)||!Number.isFinite(nowMs)||nowMs<atMs||nowMs-atMs>this.maxGapMs||!frameId){this.reset();this.state='等待单人清晰画面';return false;}
    if(this.lastTime!==null&&(atMs<=this.lastTime||atMs-this.lastTime>this.maxGapMs)){this.reset();}
    if(frameId===this.lastId)return false;
    this.lastId=frameId;this.lastTime=atMs;
    if(!this.ready){
      this.state='请先闭嘴';
      if(ratio<=this.close){this.since??=atMs;if(atMs-this.since>=this.closeMs){this.ready=true;this.since=null;this.state='等待张嘴';}}
      else this.since=null;
      return false;
    }
    if(ratio>=this.open){
      this.since??=atMs;this.state='持续张嘴确认中';
      if(atMs-this.since>=this.openMs){this.reset();this.state='触发一次';return true;}
    }else{this.since=null;this.state='等待张嘴';}
    return false;
  }
}

export function validateTrajectory(value, jointNames){
  if(value?.schema!==1||value.mode!=='simulation_only'||value.hardware_execution_enabled!==false||value.frame!=='base_link'||value.units!=='radians_seconds')throw Error('需要 simulation_only、base_link、radians_seconds 模板，且实机执行必须为 false');
  if(JSON.stringify(value.joint_names)!==JSON.stringify(jointNames))throw Error('关节名称或顺序不匹配');
  if(!Array.isArray(value.virtual_mouth_m)||value.virtual_mouth_m.length!==3||!value.virtual_mouth_m.every(Number.isFinite))throw Error('虚拟嘴部位置必须是三个有限数');
  const ids=['scoop','deliver','wait','retreat'];
  if(value.stages?.length!==4)throw Error('需要取餐、送餐、等待、撤回四段');
  let previous=null,total=0,count=0;
  for(let i=0;i<4;i++){
    const stage=value.stages[i];if(stage.id!==ids[i]||!Array.isArray(stage.samples)||stage.samples.length<2)throw Error('轨迹阶段或样本缺失');
    let time=-1;
    for(const [n,s] of stage.samples.entries()){
      if(!Number.isFinite(s.t_s)||s.t_s<=time||n===0&&s.t_s!==0||!Array.isArray(s.q_rad)||s.q_rad.length!==4||!s.q_rad.every(v=>Number.isFinite(v)&&Math.abs(v)<=Math.PI*8))throw Error('时间需从零严格递增，关节值需为四个有限弧度数');
      if(previous&&Math.max(...s.q_rad.map((v,j)=>Math.abs(v-previous[j])))>(n===0?1e-6:.25))throw Error('阶段连接不连续或相邻姿态跳变过大');
      if(stage.id==='wait'&&n>0&&s.q_rad.some((v,j)=>Math.abs(v-stage.samples[0].q_rad[j])>1e-6))throw Error('等待阶段必须保持同一姿态');
      previous=s.q_rad;time=s.t_s;count++;
    }
    total+=time;
  }
  if(count>20000||total>180||total<=0)throw Error('预演限制为 180 秒、20000 个样本');
  if(previous.some((v,j)=>Math.abs(v-value.stages[0].samples[0].q_rad[j])>1e-6))throw Error('撤回终点必须回到取餐起点');
  return value;
}

export function sampleTrajectory(plan,time){
  let offset=0;
  for(const [index,stage] of plan.stages.entries()){
    const end=stage.samples.at(-1).t_s;
    if(time<=offset+end||index===plan.stages.length-1){
      const local=Math.max(0,Math.min(end,time-offset));
      const right=stage.samples.findIndex(s=>s.t_s>=local),b=stage.samples[right<0?stage.samples.length-1:right],a=stage.samples[Math.max(0,right-1)];
      const u=b.t_s===a.t_s?0:(local-a.t_s)/(b.t_s-a.t_s);
      return {stage,index,local,q:a.q_rad.map((v,j)=>v+(b.q_rad[j]-v)*u)};
    }offset+=end;
  }
}
