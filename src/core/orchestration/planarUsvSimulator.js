import { createPlanarUsvClassicalController } from '../controllers/planarUsvClassicalController.js';
import {
  createPlanarUsvConfig,
  stepPlanarUsv,
  wrapUsvAngle,
} from '../models/planarUsv.js';

export const defaultPlanarUsvScenario={
  dt:0.02,
  duration:35,
  targetSpeed:1.8,
  path:{amplitude:2.5,wavelength:55},
  plant:{
    mass:18,
    yawInertia:8,
    surgeLinearDrag:1.8,
    surgeQuadraticDrag:5.2,
    swayLinearDrag:7,
    swayQuadraticDrag:10,
    yawLinearDrag:4.5,
    yawQuadraticDrag:3.5,
    minSurgeForce:-30,
    maxSurgeForce:45,
    maxYawMoment:18,
  },
  actuator:{
    maxSurgeForceRate:90,
    maxYawMomentRate:55,
  },
  controller:{
    speedKp:20,
    speedKi:3.5,
    speedKd:0.5,
    headingKp:25,
    headingKd:11,
    crossTrackGain:1.15,
    softening:0.8,
  },
  initialState:{x:0,y:0.9,psi:0,u:0,v:0,r:0},
  safety:{
    maxAbsCrossTrack:2.0,
    maxAbsHeadingError:0.85,
    maxSpeed:2.8,
    maxAbsYawRate:0.65,
  },
};

const sq=(x)=>x*x;

function mergeScenario(cfg={}){
  return {
    ...defaultPlanarUsvScenario,
    ...cfg,
    path:{...defaultPlanarUsvScenario.path,...(cfg.path||{})},
    plant:{...defaultPlanarUsvScenario.plant,...(cfg.plant||{})},
    actuator:{...defaultPlanarUsvScenario.actuator,...(cfg.actuator||{})},
    controller:{...defaultPlanarUsvScenario.controller,...(cfg.controller||{})},
    initialState:{...defaultPlanarUsvScenario.initialState,...(cfg.initialState||{})},
    safety:{...defaultPlanarUsvScenario.safety,...(cfg.safety||{})},
  };
}

export function planarUsvPathReference(x,cfg){
  const A=cfg.path.amplitude;
  const k=2*Math.PI/cfg.path.wavelength;
  const y=A*Math.sin(k*x);
  const dydx=A*k*Math.cos(k*x);
  return {x,y,heading:Math.atan(dydx),speed:cfg.targetSpeed};
}

function safetyViolation(state,reference,command,cfg){
  const cte=reference.y-state.y;
  const heading=wrapUsvAngle(reference.heading-state.psi);
  const speed=Math.hypot(state.u,state.v);
  return Math.max(
    0,
    Math.abs(cte)-cfg.safety.maxAbsCrossTrack,
    Math.abs(heading)-cfg.safety.maxAbsHeadingError,
    speed-cfg.safety.maxSpeed,
    Math.abs(state.r)-cfg.safety.maxAbsYawRate,
    command.surgeForce-cfg.plant.maxSurgeForce,
    cfg.plant.minSurgeForce-command.surgeForce,
    Math.abs(command.yawMoment)-cfg.plant.maxYawMoment,
  );
}

export function runPlanarUsvClassicalBaseline(userCfg={}){
  const cfg=mergeScenario(userCfg);
  createPlanarUsvConfig({...cfg.plant,dt:cfg.dt});
  const controller=createPlanarUsvClassicalController({
    ...cfg.controller,
    ...cfg.plant,
    ...cfg.actuator,
    dt:cfg.dt,
  });

  let state={...cfg.initialState};
  let previousCommand={surgeForce:0,yawMoment:0};
  const samples=[];
  const steps=Math.floor(cfg.duration/cfg.dt);
  let sumCte2=0;
  let sumHeading2=0;
  let sumSpeed2=0;
  let sumEffort=0;
  let maxCte=0;
  let maxHeading=0;
  let maxSpeed=0;
  let maxYawRate=0;
  let maxForce=0;
  let maxMoment=0;
  let maxForceRate=0;
  let maxMomentRate=0;
  let unsafeSamples=0;
  let maxViolation=0;

  const start=performance.now();
  for(let k=0;k<=steps;k+=1){
    const t=k*cfg.dt;
    const reference=planarUsvPathReference(state.x,cfg);
    const command=controller.update(state,reference);
    const forceRate=(command.surgeForce-previousCommand.surgeForce)/cfg.dt;
    const momentRate=(command.yawMoment-previousCommand.yawMoment)/cfg.dt;
    previousCommand={surgeForce:command.surgeForce,yawMoment:command.yawMoment};

    const cte=reference.y-state.y;
    const headingError=wrapUsvAngle(reference.heading-state.psi);
    const speedError=reference.speed-state.u;
    const speed=Math.hypot(state.u,state.v);
    const violation=safetyViolation(state,reference,command,cfg);

    sumCte2+=sq(cte);
    sumHeading2+=sq(headingError);
    sumSpeed2+=sq(speedError);
    sumEffort+=(
      sq(command.surgeForce/cfg.plant.maxSurgeForce)
      +0.2*sq(command.yawMoment/cfg.plant.maxYawMoment)
    )*cfg.dt;

    maxCte=Math.max(maxCte,Math.abs(cte));
    maxHeading=Math.max(maxHeading,Math.abs(headingError));
    maxSpeed=Math.max(maxSpeed,speed);
    maxYawRate=Math.max(maxYawRate,Math.abs(state.r));
    maxForce=Math.max(maxForce,Math.abs(command.surgeForce));
    maxMoment=Math.max(maxMoment,Math.abs(command.yawMoment));
    maxForceRate=Math.max(maxForceRate,Math.abs(forceRate));
    maxMomentRate=Math.max(maxMomentRate,Math.abs(momentRate));
    if(violation>0) unsafeSamples+=1;
    maxViolation=Math.max(maxViolation,violation);

    samples.push({
      t,
      ...state,
      refY:reference.y,
      refHeading:reference.heading,
      refSpeed:reference.speed,
      cte,
      headingError,
      speedError,
      speed,
      desiredHeading:command.desiredHeading,
      surgeForce:command.surgeForce,
      yawMoment:command.yawMoment,
      surgeForceRate:forceRate,
      yawMomentRate:momentRate,
      safetyViolation:violation,
    });

    state=stepPlanarUsv(state,command,{...cfg.plant,dt:cfg.dt});
  }

  const elapsedMs=performance.now()-start;
  const n=samples.length;
  return {
    samples,
    config:cfg,
    metrics:{
      crossTrackRmse:Math.sqrt(sumCte2/n),
      headingRmse:Math.sqrt(sumHeading2/n),
      speedRmse:Math.sqrt(sumSpeed2/n),
      maxAbsCrossTrack:maxCte,
      maxAbsHeadingError:maxHeading,
      maxSpeed,
      maxAbsYawRate:maxYawRate,
      maxSurgeForce:maxForce,
      maxYawMoment:maxMoment,
      maxSurgeForceRate:maxForceRate,
      maxYawMomentRate:maxMomentRate,
      controlEffort:sumEffort,
      unsafeSamples,
      unsafeRate:100*unsafeSamples/n,
      maxViolation,
      finalX:state.x,
      finalY:state.y,
      finalPsi:state.psi,
      finalSurgeSpeed:state.u,
      finalSwaySpeed:state.v,
      finalYawRate:state.r,
      simulationMs:elapsedMs,
      computePerStepUs:1000*elapsedMs/n,
    },
  };
}
