import {
  assertBicycleConfigContract,
  assertFiniteRecord,
} from '../../contracts/controlContracts.js';
const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
export function wrapAngle(a){let x=a;while(x>Math.PI)x-=2*Math.PI;while(x<-Math.PI)x+=2*Math.PI;return x;}
export function createBicycleConfig(cfg={}){
  return assertBicycleConfigContract({
    dt:cfg.dt??0.02,
    wheelbase:cfg.wheelbase??2.7,
    maxSteer:cfg.maxSteer??0.55,
    maxSteerRate:cfg.maxSteerRate??0.9,
    minSpeed:cfg.minSpeed??0,
    maxSpeed:cfg.maxSpeed??8,
    minAccel:cfg.minAccel??-3,
    maxAccel:cfg.maxAccel??2.5,
  });
}
export function stepKinematicBicycle(state,input,cfg={}){
  assertFiniteRecord(state,['x','y','yaw','v'],'bicycle.state');
  assertFiniteRecord(input,['steer','accel'],'bicycle.input');
  const p=createBicycleConfig(cfg);
  const steer=clamp(input.steer,-p.maxSteer,p.maxSteer);
  const accel=clamp(input.accel,p.minAccel,p.maxAccel);
  const v=clamp(state.v,p.minSpeed,p.maxSpeed);
  const dt=p.dt;
  return {
    x:state.x+dt*v*Math.cos(state.yaw),
    y:state.y+dt*v*Math.sin(state.yaw),
    yaw:wrapAngle(state.yaw+dt*(v/p.wheelbase)*Math.tan(steer)),
    v:clamp(v+dt*accel,p.minSpeed,p.maxSpeed),
  };
}
export function bicycleDerivatives(state,input,cfg={}){
  assertFiniteRecord(state,['x','y','yaw','v'],'bicycle.state');
  assertFiniteRecord(input,['steer','accel'],'bicycle.input');
  const p=createBicycleConfig(cfg);
  const steer=clamp(input.steer,-p.maxSteer,p.maxSteer);
  const accel=clamp(input.accel,p.minAccel,p.maxAccel);
  return {
    xDot:state.v*Math.cos(state.yaw),
    yDot:state.v*Math.sin(state.yaw),
    yawDot:(state.v/p.wheelbase)*Math.tan(steer),
    vDot:accel,
  };
}
