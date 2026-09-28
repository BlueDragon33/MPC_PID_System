import { createPlanarUavClassicalController } from '../controllers/planarUavClassicalController.js';
import { createPlanarUavConfig, stepPlanarUav, wrapPlanarAngle } from '../models/planarUav.js';

export const defaultPlanarUavScenario={
  dt:0.01,
  duration:18,
  reference:{
    forwardSpeed:1.1,
    altitude:1.9,
    altitudeAmplitude:0.22,
    altitudeFrequency:0.38,
  },
  plant:{
    mass:1.4,
    inertia:0.035,
    gravity:9.81,
    linearDragX:0.16,
    linearDragZ:0.22,
    angularDamping:0.08,
    minThrust:0,
    maxThrust:30.22,
    maxTorque:0.7,
  },
  controller:{
    xP:1.2,
    xD:1.8,
    zP:2.4,
    zD:2.0,
    thetaP:9.0,
    thetaD:2.2,
    maxTilt:0.45,
  },
  initialState:{x:0,z:1.55,theta:0,vx:0,vz:0,q:0},
  safety:{
    maxAbsXError:0.9,
    maxAbsZError:0.8,
    maxAbsTilt:0.55,
    maxAbsVx:3.5,
    maxAbsVz:2.5,
    minAltitude:0.5,
    maxAltitude:3.2,
  },
};

const sq=(x)=>x*x;

function mergeScenario(cfg={}){
  return {
    ...defaultPlanarUavScenario,
    ...cfg,
    reference:{...defaultPlanarUavScenario.reference,...(cfg.reference||{})},
    plant:{...defaultPlanarUavScenario.plant,...(cfg.plant||{})},
    controller:{...defaultPlanarUavScenario.controller,...(cfg.controller||{})},
    initialState:{...defaultPlanarUavScenario.initialState,...(cfg.initialState||{})},
    safety:{...defaultPlanarUavScenario.safety,...(cfg.safety||{})},
  };
}

export function planarUavReference(t,cfg){
  const r=cfg.reference;
  return {
    x:r.forwardSpeed*t,
    vx:r.forwardSpeed,
    z:r.altitude+r.altitudeAmplitude*Math.sin(r.altitudeFrequency*t),
    vz:r.altitudeAmplitude*r.altitudeFrequency*Math.cos(r.altitudeFrequency*t),
  };
}

function safetyViolation(state,reference,cfg){
  const s=cfg.safety;
  return Math.max(
    0,
    Math.abs(reference.x-state.x)-s.maxAbsXError,
    Math.abs(reference.z-state.z)-s.maxAbsZError,
    Math.abs(state.theta)-s.maxAbsTilt,
    Math.abs(state.vx)-s.maxAbsVx,
    Math.abs(state.vz)-s.maxAbsVz,
    s.minAltitude-state.z,
    state.z-s.maxAltitude,
  );
}

export function runPlanarUavClassicalBaseline(userCfg={}){
  const cfg=mergeScenario(userCfg);
  createPlanarUavConfig({...cfg.plant,dt:cfg.dt});
  const controller=createPlanarUavClassicalController({
    ...cfg.controller,
    ...cfg.plant,
  });

  let state={...cfg.initialState};
  const samples=[];
  const steps=Math.floor(cfg.duration/cfg.dt);
  let sumX2=0,sumZ2=0,sumVx2=0,sumVz2=0,sumAttitude2=0,sumEffort=0;
  let maxAbsXError=0,maxAbsZError=0,maxAbsTilt=0,maxAbsVx=0,maxAbsVz=0,unsafeSamples=0;
  let maxThrust=0,maxTorque=0;

  const start=performance.now();
  for(let k=0;k<=steps;k+=1){
    const t=k*cfg.dt;
    const reference=planarUavReference(t,cfg);
    const command=controller.update(state,reference);
    const xError=reference.x-state.x;
    const zError=reference.z-state.z;
    const vxError=reference.vx-state.vx;
    const vzError=reference.vz-state.vz;
    const attitudeError=wrapPlanarAngle(command.desiredTheta-state.theta);
    const violation=safetyViolation(state,reference,cfg);

    sumX2+=sq(xError);
    sumZ2+=sq(zError);
    sumVx2+=sq(vxError);
    sumVz2+=sq(vzError);
    sumAttitude2+=sq(attitudeError);
    const hoverThrust=cfg.plant.mass*cfg.plant.gravity;
    sumEffort+=sq(command.thrust/hoverThrust-1)+0.05*sq(command.torque/cfg.plant.maxTorque);

    maxAbsXError=Math.max(maxAbsXError,Math.abs(xError));
    maxAbsZError=Math.max(maxAbsZError,Math.abs(zError));
    maxAbsTilt=Math.max(maxAbsTilt,Math.abs(state.theta));
    maxAbsVx=Math.max(maxAbsVx,Math.abs(state.vx));
    maxAbsVz=Math.max(maxAbsVz,Math.abs(state.vz));
    maxThrust=Math.max(maxThrust,command.thrust);
    maxTorque=Math.max(maxTorque,Math.abs(command.torque));
    if(violation>0)unsafeSamples+=1;

    samples.push({
      t,
      ...state,
      refX:reference.x,
      refZ:reference.z,
      refVx:reference.vx,
      refVz:reference.vz,
      xError,
      zError,
      vxError,
      vzError,
      desiredTheta:command.desiredTheta,
      attitudeError,
      thrust:command.thrust,
      torque:command.torque,
      safetyViolation:violation,
    });

    state=stepPlanarUav(state,command,{...cfg.plant,dt:cfg.dt});
  }

  const elapsedMs=performance.now()-start;
  const n=samples.length;
  return {
    samples,
    config:cfg,
    metrics:{
      xRmse:Math.sqrt(sumX2/n),
      zRmse:Math.sqrt(sumZ2/n),
      vxRmse:Math.sqrt(sumVx2/n),
      vzRmse:Math.sqrt(sumVz2/n),
      attitudeTrackingRmse:Math.sqrt(sumAttitude2/n),
      maxAbsXError,
      maxAbsZError,
      maxAbsTilt,
      maxAbsVx,
      maxAbsVz,
      maxThrust,
      maxTorque,
      controlEffort:sumEffort*cfg.dt,
      unsafeSamples,
      unsafeRate:100*unsafeSamples/n,
      finalX:state.x,
      finalZ:state.z,
      finalVx:state.vx,
      finalVz:state.vz,
      finalTheta:state.theta,
      simulationMs:elapsedMs,
      computePerStepUs:1000*elapsedMs/n,
    },
  };
}
