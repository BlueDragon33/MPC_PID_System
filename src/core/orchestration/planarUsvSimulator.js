import { createPlanarUsvClassicalController } from '../controllers/planarUsvClassicalController.js';
import { createPlanarUsvEkf } from '../estimation/planarUsvEkf.js';
import { createPlanarUsvMeasurementSensor } from '../estimation/planarUsvMeasurementSensor.js';
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
  estimation:{
    enabled:false,
    seed:20260929,
    noiseStd:{x:0.12,y:0.12,psi:0.02,u:0.08,v:0.05,r:0.025},
    processCovariance:[1e-5,1e-5,1e-6,2e-4,2e-4,1e-4],
    initialCovariance:[0.2,0.2,0.04,0.15,0.08,0.05],
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
    estimation:{
      ...defaultPlanarUsvScenario.estimation,
      ...(cfg.estimation||{}),
      noiseStd:{
        ...defaultPlanarUsvScenario.estimation.noiseStd,
        ...(cfg.estimation?.noiseStd||{}),
      },
    },
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

function createEstimator(cfg,state){
  if(!cfg.estimation.enabled){
    return {
      enabled:false,
      sensor:null,
      ekf:null,
      measurement:null,
      estimate:{...state,covariance:null,diagnostics:null},
      controllerState:{...state},
    };
  }

  const sensor=createPlanarUsvMeasurementSensor({
    seed:cfg.estimation.seed,
    noiseStd:cfg.estimation.noiseStd,
  });
  const measurement=sensor.read(state);
  const ekf=createPlanarUsvEkf({
    cfg:{...cfg.plant,dt:cfg.dt},
    initialState:[
      measurement.x,
      measurement.y,
      measurement.psi,
      measurement.u,
      measurement.v,
      measurement.r,
    ],
    initialCovariance:cfg.estimation.initialCovariance,
    processCovariance:cfg.estimation.processCovariance,
    measurementVariance:[
      cfg.estimation.noiseStd.x**2,
      cfg.estimation.noiseStd.y**2,
      cfg.estimation.noiseStd.psi**2,
      cfg.estimation.noiseStd.u**2,
      cfg.estimation.noiseStd.v**2,
      cfg.estimation.noiseStd.r**2,
    ],
  });
  const estimate=ekf.update(measurement);
  return {
    enabled:true,
    sensor,
    ekf,
    measurement,
    estimate,
    controllerState:{
      x:estimate.x,
      y:estimate.y,
      psi:estimate.psi,
      u:estimate.u,
      v:estimate.v,
      r:estimate.r,
    },
  };
}

function createStateErrorAccumulator(){
  return {x:0,y:0,psi:0,u:0,v:0,r:0};
}

function addMeasurementError(errors,measurement,state){
  errors.x+=sq(measurement.x-state.x);
  errors.y+=sq(measurement.y-state.y);
  errors.psi+=sq(wrapUsvAngle(measurement.psi-state.psi));
  errors.u+=sq(measurement.u-state.u);
  errors.v+=sq(measurement.v-state.v);
  errors.r+=sq(measurement.r-state.r);
}

function addEstimateError(errors,estimate,state){
  errors.x+=sq(estimate.x-state.x);
  errors.y+=sq(estimate.y-state.y);
  errors.psi+=sq(wrapUsvAngle(estimate.psi-state.psi));
  errors.u+=sq(estimate.u-state.u);
  errors.v+=sq(estimate.v-state.v);
  errors.r+=sq(estimate.r-state.r);
}

function stateRmse(errors,n){
  return {
    x:Math.sqrt(errors.x/n),
    y:Math.sqrt(errors.y/n),
    psi:Math.sqrt(errors.psi/n),
    u:Math.sqrt(errors.u/n),
    v:Math.sqrt(errors.v/n),
    r:Math.sqrt(errors.r/n),
  };
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
  let estimator=createEstimator(cfg,state);
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
  const measurementError2=createStateErrorAccumulator();
  const estimateError2=createStateErrorAccumulator();
  let covarianceTraceSum=0;

  const start=performance.now();
  for(let k=0;k<=steps;k+=1){
    const t=k*cfg.dt;
    const controlReference=planarUsvPathReference(estimator.controllerState.x,cfg);
    const command=controller.update(estimator.controllerState,controlReference);
    const forceRate=(command.surgeForce-previousCommand.surgeForce)/cfg.dt;
    const momentRate=(command.yawMoment-previousCommand.yawMoment)/cfg.dt;
    previousCommand={surgeForce:command.surgeForce,yawMoment:command.yawMoment};

    const auditReference=planarUsvPathReference(state.x,cfg);
    const cte=auditReference.y-state.y;
    const headingError=wrapUsvAngle(auditReference.heading-state.psi);
    const speedError=auditReference.speed-state.u;
    const speed=Math.hypot(state.u,state.v);
    const violation=safetyViolation(state,auditReference,command,cfg);

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

    if(estimator.enabled){
      addMeasurementError(measurementError2,estimator.measurement,state);
      addEstimateError(estimateError2,estimator.controllerState,state);
      covarianceTraceSum+=estimator.estimate.diagnostics?.covarianceTrace??0;
    }

    samples.push({
      t,
      ...state,
      controllerX:estimator.controllerState.x,
      controllerY:estimator.controllerState.y,
      controllerPsi:estimator.controllerState.psi,
      controllerU:estimator.controllerState.u,
      controllerV:estimator.controllerState.v,
      controllerR:estimator.controllerState.r,
      refY:auditReference.y,
      refHeading:auditReference.heading,
      refSpeed:auditReference.speed,
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
      estimationEnabled:estimator.enabled,
      measurementX:estimator.measurement?.x??null,
      measurementY:estimator.measurement?.y??null,
      measurementPsi:estimator.measurement?.psi??null,
      measurementU:estimator.measurement?.u??null,
      measurementV:estimator.measurement?.v??null,
      measurementR:estimator.measurement?.r??null,
      covarianceTrace:estimator.enabled
        ?(estimator.estimate.diagnostics?.covarianceTrace??null)
        :null,
      innovations:estimator.enabled
        ?(estimator.estimate.diagnostics?.innovations??null)
        :null,
    });

    state=stepPlanarUsv(state,command,{...cfg.plant,dt:cfg.dt});
    if(estimator.enabled){
      const measurement=estimator.sensor.read(state);
      const estimate=estimator.ekf.step(command,measurement);
      estimator={
        ...estimator,
        measurement,
        estimate,
        controllerState:{
          x:estimate.x,
          y:estimate.y,
          psi:estimate.psi,
          u:estimate.u,
          v:estimate.v,
          r:estimate.r,
        },
      };
    }else{
      estimator={...estimator,controllerState:{...state}};
    }
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
      estimationEnabled:estimator.enabled,
      measurementRmse:estimator.enabled
        ?stateRmse(measurementError2,n)
        :null,
      estimateRmse:estimator.enabled
        ?stateRmse(estimateError2,n)
        :null,
      averageCovarianceTrace:estimator.enabled
        ?covarianceTraceSum/n
        :null,
    },
  };
}
