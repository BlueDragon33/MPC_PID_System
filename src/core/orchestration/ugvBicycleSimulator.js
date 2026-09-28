import { createUGVPathFollower } from '../controllers/ugvPathFollower.js';
import { createBicycleConfig, stepKinematicBicycle, wrapAngle } from '../models/kinematicBicycle.js';

export const defaultUGVScenario={
  dt:0.02,duration:22,targetSpeed:4.0,
  path:{amplitude:1.2,wavelength:34},
  plant:{wheelbase:2.7,maxSteer:0.55,maxSteerRate:0.9,minSpeed:0,maxSpeed:8,minAccel:-3,maxAccel:2.5},
  controller:{stanleyGain:1.35,softening:0.8,speedKp:1.8,speedKi:0.35,speedKd:0.08},
  safety:{maxAbsCrossTrack:1.25,maxAbsHeadingError:0.8}
};
const sq=x=>x*x;
function merge(cfg={}){return {...defaultUGVScenario,...cfg,path:{...defaultUGVScenario.path,...(cfg.path||{})},plant:{...defaultUGVScenario.plant,...(cfg.plant||{})},controller:{...defaultUGVScenario.controller,...(cfg.controller||{})},safety:{...defaultUGVScenario.safety,...(cfg.safety||{})}};}
export function ugvPathReference(x,cfg){
  const A=cfg.path.amplitude,k=2*Math.PI/cfg.path.wavelength;
  const y=A*Math.sin(k*x);
  const dydx=A*k*Math.cos(k*x);
  return {x,y,heading:Math.atan(dydx),speed:cfg.targetSpeed};
}
export function runUGVBicycleBaseline(userCfg={}){
  const cfg=merge(userCfg); const p=createBicycleConfig({...cfg.plant,dt:cfg.dt});
  const controller=createUGVPathFollower({...cfg.controller,...cfg.plant,dt:cfg.dt});
  let state={x:0,y:0.65,yaw:0,v:0};
  let prevSteer=0; const samples=[]; const steps=Math.floor(cfg.duration/cfg.dt);
  let sumCte2=0,sumHeading2=0,sumSpeed2=0,sumEffort=0,maxCte=0,maxHeading=0,maxSteerRate=0,unsafe=0;
  const start=performance.now();
  for(let k=0;k<=steps;k++){
    const t=k*cfg.dt; const ref=ugvPathReference(state.x,cfg); const cmd=controller.update(state,ref);
    const steerRate=(cmd.steer-prevSteer)/cfg.dt; prevSteer=cmd.steer;
    const headingError=wrapAngle(ref.heading-state.yaw); const cte=ref.y-state.y; const speedError=ref.speed-state.v;
    maxCte=Math.max(maxCte,Math.abs(cte)); maxHeading=Math.max(maxHeading,Math.abs(headingError)); maxSteerRate=Math.max(maxSteerRate,Math.abs(steerRate));
    sumCte2+=sq(cte);sumHeading2+=sq(headingError);sumSpeed2+=sq(speedError);sumEffort+=sq(cmd.steer)+0.12*sq(cmd.accel);
    const safetyViolation=Math.max(0,Math.abs(cte)-cfg.safety.maxAbsCrossTrack,Math.abs(headingError)-cfg.safety.maxAbsHeadingError);
    if(safetyViolation>0) unsafe++;
    samples.push({t,...state,refY:ref.y,refHeading:ref.heading,refSpeed:ref.speed,cte,headingError,speedError,steer:cmd.steer,accel:cmd.accel,steerRate,safetyViolation});
    state=stepKinematicBicycle(state,cmd,{...cfg.plant,dt:cfg.dt});
  }
  const elapsedMs=performance.now()-start,n=samples.length;
  return {samples,config:cfg,metrics:{
    crossTrackRmse:Math.sqrt(sumCte2/n),headingRmse:Math.sqrt(sumHeading2/n),speedRmse:Math.sqrt(sumSpeed2/n),
    maxAbsCrossTrack:maxCte,maxAbsHeadingError:maxHeading,maxSteerRate,controlEffort:sumEffort*cfg.dt,
    unsafeSamples:unsafe,unsafeRate:100*unsafe/n,finalSpeed:state.v,finalX:state.x,
    simulationMs:elapsedMs,computePerStepUs:1000*elapsedMs/n
  }};
}
