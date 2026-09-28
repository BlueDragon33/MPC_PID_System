import { createPlanarUavClassicalController } from '../controllers/planarUavClassicalController.js';
import { createPlanarUavEkf } from '../estimation/planarUavEkf.js';
import { createPlanarUavMeasurementSensor } from '../estimation/planarUavMeasurementSensor.js';
import { createPlanarUavConfig, stepPlanarUav, wrapPlanarAngle } from '../models/planarUav.js';

export const defaultPlanarUavScenario={
  dt:0.01,duration:18,
  reference:{forwardSpeed:1.1,altitude:1.9,altitudeAmplitude:0.22,altitudeFrequency:0.38},
  plant:{mass:1.4,inertia:0.035,gravity:9.81,linearDragX:0.16,linearDragZ:0.22,angularDamping:0.08,minThrust:0,maxThrust:30.22,maxTorque:0.7},
  controller:{xP:1.2,xD:1.8,zP:2.4,zD:2.0,thetaP:9.0,thetaD:2.2,maxTilt:0.45},
  estimation:{enabled:false,seed:20260928,noiseStd:{x:0.08,z:0.06,theta:0.015,vx:0.08,vz:0.07,q:0.04},processCovariance:[1e-5,1e-5,1e-6,3e-4,3e-4,4e-4],initialCovariance:[0.12,0.10,0.03,0.15,0.15,0.08]},
  initialState:{x:0,z:1.55,theta:0,vx:0,vz:0,q:0},
  safety:{maxAbsXError:0.9,maxAbsZError:0.8,maxAbsTilt:0.55,maxAbsVx:3.5,maxAbsVz:2.5,minAltitude:0.5,maxAltitude:3.2},
};
const sq=x=>x*x;
function mergeScenario(cfg={}){return {...defaultPlanarUavScenario,...cfg,reference:{...defaultPlanarUavScenario.reference,...(cfg.reference||{})},plant:{...defaultPlanarUavScenario.plant,...(cfg.plant||{})},controller:{...defaultPlanarUavScenario.controller,...(cfg.controller||{})},estimation:{...defaultPlanarUavScenario.estimation,...(cfg.estimation||{}),noiseStd:{...defaultPlanarUavScenario.estimation.noiseStd,...(cfg.estimation?.noiseStd||{})}},initialState:{...defaultPlanarUavScenario.initialState,...(cfg.initialState||{})},safety:{...defaultPlanarUavScenario.safety,...(cfg.safety||{})}};}
export function planarUavReference(t,cfg){const r=cfg.reference;return {x:r.forwardSpeed*t,vx:r.forwardSpeed,z:r.altitude+r.altitudeAmplitude*Math.sin(r.altitudeFrequency*t),vz:r.altitudeAmplitude*r.altitudeFrequency*Math.cos(r.altitudeFrequency*t)};}
function safetyViolation(state,reference,cfg){const s=cfg.safety;return Math.max(0,Math.abs(reference.x-state.x)-s.maxAbsXError,Math.abs(reference.z-state.z)-s.maxAbsZError,Math.abs(state.theta)-s.maxAbsTilt,Math.abs(state.vx)-s.maxAbsVx,Math.abs(state.vz)-s.maxAbsVz,s.minAltitude-state.z,state.z-s.maxAltitude);}
export function runPlanarUavClassicalBaseline(userCfg={}){
  const cfg=mergeScenario(userCfg);createPlanarUavConfig({...cfg.plant,dt:cfg.dt});
  const controller=createPlanarUavClassicalController({...cfg.controller,...cfg.plant});
  let state={...cfg.initialState};
  const estimationEnabled=Boolean(cfg.estimation.enabled);
  const sensor=estimationEnabled?createPlanarUavMeasurementSensor({seed:cfg.estimation.seed,noiseStd:cfg.estimation.noiseStd}):null;
  let measurement=estimationEnabled?sensor.read(state):null;
  const ekf=estimationEnabled?createPlanarUavEkf({cfg:{...cfg.plant,dt:cfg.dt},initialState:[measurement.x,measurement.z,measurement.theta,measurement.vx,measurement.vz,measurement.q],initialCovariance:cfg.estimation.initialCovariance,processCovariance:cfg.estimation.processCovariance,measurementVariance:[cfg.estimation.noiseStd.x**2,cfg.estimation.noiseStd.z**2,cfg.estimation.noiseStd.theta**2,cfg.estimation.noiseStd.vx**2,cfg.estimation.noiseStd.vz**2,cfg.estimation.noiseStd.q**2]}):null;
  let estimate=estimationEnabled?ekf.update(measurement):{...state,covariance:null,diagnostics:null};
  let controllerState=estimationEnabled?{x:estimate.x,z:estimate.z,theta:estimate.theta,vx:estimate.vx,vz:estimate.vz,q:estimate.q}:{...state};
  const samples=[];const steps=Math.floor(cfg.duration/cfg.dt);
  let sumX2=0,sumZ2=0,sumVx2=0,sumVz2=0,sumAttitude2=0,sumEffort=0;
  let maxAbsXError=0,maxAbsZError=0,maxAbsTilt=0,maxAbsVx=0,maxAbsVz=0,unsafeSamples=0,maxThrust=0,maxTorque=0;
  let measurementError2={x:0,z:0,theta:0,vx:0,vz:0,q:0},estimateError2={x:0,z:0,theta:0,vx:0,vz:0,q:0},covarianceTraceSum=0;
  const start=performance.now();
  for(let k=0;k<=steps;k+=1){
    const t=k*cfg.dt;const reference=planarUavReference(t,cfg);const command=controller.update(controllerState,reference);
    const xError=reference.x-state.x,zError=reference.z-state.z,vxError=reference.vx-state.vx,vzError=reference.vz-state.vz,attitudeError=wrapPlanarAngle(command.desiredTheta-state.theta),violation=safetyViolation(state,reference,cfg);
    sumX2+=sq(xError);sumZ2+=sq(zError);sumVx2+=sq(vxError);sumVz2+=sq(vzError);sumAttitude2+=sq(attitudeError);
    const hoverThrust=cfg.plant.mass*cfg.plant.gravity;sumEffort+=sq(command.thrust/hoverThrust-1)+0.05*sq(command.torque/cfg.plant.maxTorque);
    maxAbsXError=Math.max(maxAbsXError,Math.abs(xError));maxAbsZError=Math.max(maxAbsZError,Math.abs(zError));maxAbsTilt=Math.max(maxAbsTilt,Math.abs(state.theta));maxAbsVx=Math.max(maxAbsVx,Math.abs(state.vx));maxAbsVz=Math.max(maxAbsVz,Math.abs(state.vz));maxThrust=Math.max(maxThrust,command.thrust);maxTorque=Math.max(maxTorque,Math.abs(command.torque));if(violation>0)unsafeSamples+=1;
    if(estimationEnabled){
      measurementError2.x+=sq(measurement.x-state.x);measurementError2.z+=sq(measurement.z-state.z);measurementError2.theta+=sq(wrapPlanarAngle(measurement.theta-state.theta));measurementError2.vx+=sq(measurement.vx-state.vx);measurementError2.vz+=sq(measurement.vz-state.vz);measurementError2.q+=sq(measurement.q-state.q);
      estimateError2.x+=sq(controllerState.x-state.x);estimateError2.z+=sq(controllerState.z-state.z);estimateError2.theta+=sq(wrapPlanarAngle(controllerState.theta-state.theta));estimateError2.vx+=sq(controllerState.vx-state.vx);estimateError2.vz+=sq(controllerState.vz-state.vz);estimateError2.q+=sq(controllerState.q-state.q);covarianceTraceSum+=estimate.diagnostics?.covarianceTrace??0;
    }
    samples.push({t,...state,controllerX:controllerState.x,controllerZ:controllerState.z,controllerTheta:controllerState.theta,controllerVx:controllerState.vx,controllerVz:controllerState.vz,controllerQ:controllerState.q,refX:reference.x,refZ:reference.z,refVx:reference.vx,refVz:reference.vz,xError,zError,vxError,vzError,desiredTheta:command.desiredTheta,attitudeError,thrust:command.thrust,torque:command.torque,safetyViolation:violation,estimationEnabled,measurementX:measurement?.x??null,measurementZ:measurement?.z??null,measurementTheta:measurement?.theta??null,measurementVx:measurement?.vx??null,measurementVz:measurement?.vz??null,measurementQ:measurement?.q??null,covarianceTrace:estimationEnabled?(estimate.diagnostics?.covarianceTrace??null):null,innovations:estimationEnabled?(estimate.diagnostics?.innovations??null):null});
    state=stepPlanarUav(state,command,{...cfg.plant,dt:cfg.dt});
    if(estimationEnabled){measurement=sensor.read(state);estimate=ekf.step(command,measurement);controllerState={x:estimate.x,z:estimate.z,theta:estimate.theta,vx:estimate.vx,vz:estimate.vz,q:estimate.q};}else controllerState={...state};
  }
  const elapsedMs=performance.now()-start,n=samples.length;const rmse=e=>({x:Math.sqrt(e.x/n),z:Math.sqrt(e.z/n),theta:Math.sqrt(e.theta/n),vx:Math.sqrt(e.vx/n),vz:Math.sqrt(e.vz/n),q:Math.sqrt(e.q/n)});
  return {samples,config:cfg,metrics:{xRmse:Math.sqrt(sumX2/n),zRmse:Math.sqrt(sumZ2/n),vxRmse:Math.sqrt(sumVx2/n),vzRmse:Math.sqrt(sumVz2/n),attitudeTrackingRmse:Math.sqrt(sumAttitude2/n),maxAbsXError,maxAbsZError,maxAbsTilt,maxAbsVx,maxAbsVz,maxThrust,maxTorque,controlEffort:sumEffort*cfg.dt,unsafeSamples,unsafeRate:100*unsafeSamples/n,finalX:state.x,finalZ:state.z,finalVx:state.vx,finalVz:state.vz,finalTheta:state.theta,simulationMs:elapsedMs,computePerStepUs:1000*elapsedMs/n,estimationEnabled,measurementRmse:estimationEnabled?rmse(measurementError2):null,estimateRmse:estimationEnabled?rmse(estimateError2):null,averageCovarianceTrace:estimationEnabled?covarianceTraceSum/n:null}};
}
