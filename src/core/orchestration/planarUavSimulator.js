import { createPlanarUavClassicalController } from '../controllers/planarUavClassicalController.js';
import { createPlanarUavPredictiveGovernor } from '../controllers/planarUavPredictiveGovernor.js';
import { createPlanarUavEkf } from '../estimation/planarUavEkf.js';
import { createPlanarUavMeasurementSensor } from '../estimation/planarUavMeasurementSensor.js';
import {
  createPlanarUavConfig,
  stepPlanarUav,
  wrapPlanarAngle,
} from '../models/planarUav.js';

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
  estimation:{
    enabled:false,
    seed:20260928,
    noiseStd:{
      x:0.08,
      z:0.06,
      theta:0.015,
      vx:0.08,
      vz:0.07,
      q:0.04,
    },
    processCovariance:[1e-5,1e-5,1e-6,3e-4,3e-4,4e-4],
    initialCovariance:[0.12,0.10,0.03,0.15,0.15,0.08],
  },
  predictive:{
    enabled:false,
    horizon:8,
    predictionDt:0.05,
    thrustOffsets:[-0.18,0,0.18],
    torqueOffsets:[-0.08,-0.04,0,0.04,0.08],
    qX:5,
    qZ:20,
    qVx:2,
    qVz:8,
    qTheta:2,
    qQ:0.15,
    rThrust:0.5,
    rTorque:0.06,
    rProposal:0.12,
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
    estimation:{
      ...defaultPlanarUavScenario.estimation,
      ...(cfg.estimation||{}),
      noiseStd:{
        ...defaultPlanarUavScenario.estimation.noiseStd,
        ...(cfg.estimation?.noiseStd||{}),
      },
    },
    predictive:{...defaultPlanarUavScenario.predictive,...(cfg.predictive||{})},
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

  const sensor=createPlanarUavMeasurementSensor({
    seed:cfg.estimation.seed,
    noiseStd:cfg.estimation.noiseStd,
  });
  const measurement=sensor.read(state);
  const ekf=createPlanarUavEkf({
    cfg:{...cfg.plant,dt:cfg.dt},
    initialState:[
      measurement.x,
      measurement.z,
      measurement.theta,
      measurement.vx,
      measurement.vz,
      measurement.q,
    ],
    initialCovariance:cfg.estimation.initialCovariance,
    processCovariance:cfg.estimation.processCovariance,
    measurementVariance:[
      cfg.estimation.noiseStd.x**2,
      cfg.estimation.noiseStd.z**2,
      cfg.estimation.noiseStd.theta**2,
      cfg.estimation.noiseStd.vx**2,
      cfg.estimation.noiseStd.vz**2,
      cfg.estimation.noiseStd.q**2,
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
      z:estimate.z,
      theta:estimate.theta,
      vx:estimate.vx,
      vz:estimate.vz,
      q:estimate.q,
    },
  };
}

function createStateErrorAccumulator(){
  return {x:0,z:0,theta:0,vx:0,vz:0,q:0};
}

function addMeasurementError(errors,measurement,state){
  errors.x+=sq(measurement.x-state.x);
  errors.z+=sq(measurement.z-state.z);
  errors.theta+=sq(wrapPlanarAngle(measurement.theta-state.theta));
  errors.vx+=sq(measurement.vx-state.vx);
  errors.vz+=sq(measurement.vz-state.vz);
  errors.q+=sq(measurement.q-state.q);
}

function addEstimateError(errors,estimate,state){
  errors.x+=sq(estimate.x-state.x);
  errors.z+=sq(estimate.z-state.z);
  errors.theta+=sq(wrapPlanarAngle(estimate.theta-state.theta));
  errors.vx+=sq(estimate.vx-state.vx);
  errors.vz+=sq(estimate.vz-state.vz);
  errors.q+=sq(estimate.q-state.q);
}

function stateRmse(errors,n){
  return {
    x:Math.sqrt(errors.x/n),
    z:Math.sqrt(errors.z/n),
    theta:Math.sqrt(errors.theta/n),
    vx:Math.sqrt(errors.vx/n),
    vz:Math.sqrt(errors.vz/n),
    q:Math.sqrt(errors.q/n),
  };
}

export function runPlanarUavClassicalBaseline(userCfg={}){
  const cfg=mergeScenario(userCfg);
  createPlanarUavConfig({...cfg.plant,dt:cfg.dt});
  const controller=createPlanarUavClassicalController({
    ...cfg.controller,
    ...cfg.plant,
  });
  const predictiveEnabled=Boolean(cfg.predictive.enabled);
  const predictiveGovernor=predictiveEnabled?createPlanarUavPredictiveGovernor({
    ...cfg.predictive,
    ...cfg.plant,
    ...cfg.safety,
    dt:cfg.dt,
  }):null;

  let state={...cfg.initialState};
  let estimator=createEstimator(cfg,state);

  const samples=[];
  const steps=Math.floor(cfg.duration/cfg.dt);
  let sumX2=0;
  let sumZ2=0;
  let sumVx2=0;
  let sumVz2=0;
  let sumAttitude2=0;
  let sumEffort=0;
  let maxAbsXError=0;
  let maxAbsZError=0;
  let maxAbsTilt=0;
  let maxAbsVx=0;
  let maxAbsVz=0;
  let unsafeSamples=0;
  let maxThrust=0;
  let maxTorque=0;
  const measurementError2=createStateErrorAccumulator();
  const estimateError2=createStateErrorAccumulator();
  let covarianceTraceSum=0;
  let predictiveSolveMsSum=0;
  let predictiveSolveMsMax=0;
  let predictiveFeasibleCount=0;
  let predictiveFallbackCount=0;
  let predictiveInterventions=0;
  let predictedMaxXError=0;
  let predictedMaxZError=0;
  let predictedMaxTilt=0;
  let predictedMaxVx=0;
  let predictedMaxVz=0;

  const start=performance.now();
  for(let k=0;k<=steps;k+=1){
    const t=k*cfg.dt;
    const reference=planarUavReference(t,cfg);
    const proposal=controller.update(estimator.controllerState,reference);
    let command=proposal;
    let predictiveDiagnostics=null;
    if(predictiveEnabled){
      const selected=predictiveGovernor.select({
        state:estimator.controllerState,
        time:t,
        proposal,
        referenceAt:(futureTime)=>planarUavReference(futureTime,cfg),
        baselineAt:(rolloutState,rolloutReference)=>controller.update(rolloutState,rolloutReference),
        plantCfg:cfg.plant,
      });
      command={...proposal,...selected.command};
      predictiveDiagnostics=selected.diagnostics;
      predictiveSolveMsSum+=predictiveDiagnostics.solveMs;
      predictiveSolveMsMax=Math.max(predictiveSolveMsMax,predictiveDiagnostics.solveMs);
      if(predictiveDiagnostics.predictedFeasible) predictiveFeasibleCount+=1;
      if(predictiveDiagnostics.fallbackUsed) predictiveFallbackCount+=1;
      if(
        Math.abs(command.thrust-proposal.thrust)>1e-12 ||
        Math.abs(command.torque-proposal.torque)>1e-12
      ) predictiveInterventions+=1;
      predictedMaxXError=Math.max(predictedMaxXError,predictiveDiagnostics.predictedMaxXError??0);
      predictedMaxZError=Math.max(predictedMaxZError,predictiveDiagnostics.predictedMaxZError??0);
      predictedMaxTilt=Math.max(predictedMaxTilt,predictiveDiagnostics.predictedMaxTilt??0);
      predictedMaxVx=Math.max(predictedMaxVx,predictiveDiagnostics.predictedMaxVx??0);
      predictedMaxVz=Math.max(predictedMaxVz,predictiveDiagnostics.predictedMaxVz??0);
    }
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
    sumEffort+=sq(command.thrust/hoverThrust-1)
      +0.05*sq(command.torque/cfg.plant.maxTorque);

    maxAbsXError=Math.max(maxAbsXError,Math.abs(xError));
    maxAbsZError=Math.max(maxAbsZError,Math.abs(zError));
    maxAbsTilt=Math.max(maxAbsTilt,Math.abs(state.theta));
    maxAbsVx=Math.max(maxAbsVx,Math.abs(state.vx));
    maxAbsVz=Math.max(maxAbsVz,Math.abs(state.vz));
    maxThrust=Math.max(maxThrust,command.thrust);
    maxTorque=Math.max(maxTorque,Math.abs(command.torque));
    if(violation>0)unsafeSamples+=1;

    if(estimator.enabled){
      addMeasurementError(measurementError2,estimator.measurement,state);
      addEstimateError(estimateError2,estimator.controllerState,state);
      covarianceTraceSum+=estimator.estimate.diagnostics?.covarianceTrace??0;
    }

    samples.push({
      t,
      ...state,
      controllerX:estimator.controllerState.x,
      controllerZ:estimator.controllerState.z,
      controllerTheta:estimator.controllerState.theta,
      controllerVx:estimator.controllerState.vx,
      controllerVz:estimator.controllerState.vz,
      controllerQ:estimator.controllerState.q,
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
      proposalThrust:proposal.thrust,
      proposalTorque:proposal.torque,
      thrust:command.thrust,
      torque:command.torque,
      safetyViolation:violation,
      predictiveEnabled,
      predictiveFeasible:predictiveDiagnostics?.predictedFeasible??null,
      predictiveFallbackUsed:predictiveDiagnostics?.fallbackUsed??false,
      predictiveCost:predictiveDiagnostics?.cost??null,
      predictiveSolveMs:predictiveDiagnostics?.solveMs??null,
      predictiveThrustOffset:predictiveDiagnostics?.thrustOffset??null,
      predictiveTorqueOffset:predictiveDiagnostics?.torqueOffset??null,
      predictedMaxXError:predictiveDiagnostics?.predictedMaxXError??null,
      predictedMaxZError:predictiveDiagnostics?.predictedMaxZError??null,
      predictedMaxTilt:predictiveDiagnostics?.predictedMaxTilt??null,
      predictedMaxVx:predictiveDiagnostics?.predictedMaxVx??null,
      predictedMaxVz:predictiveDiagnostics?.predictedMaxVz??null,
      predictiveCandidateCount:predictiveDiagnostics?.candidateCount??null,
      predictiveFeasibleCount:predictiveDiagnostics?.feasibleCount??null,
      estimationEnabled:estimator.enabled,
      measurementX:estimator.measurement?.x??null,
      measurementZ:estimator.measurement?.z??null,
      measurementTheta:estimator.measurement?.theta??null,
      measurementVx:estimator.measurement?.vx??null,
      measurementVz:estimator.measurement?.vz??null,
      measurementQ:estimator.measurement?.q??null,
      covarianceTrace:estimator.enabled
        ?(estimator.estimate.diagnostics?.covarianceTrace??null)
        :null,
      innovations:estimator.enabled
        ?(estimator.estimate.diagnostics?.innovations??null)
        :null,
    });

    state=stepPlanarUav(state,command,{...cfg.plant,dt:cfg.dt});
    if(estimator.enabled){
      const measurement=estimator.sensor.read(state);
      const estimate=estimator.ekf.step(command,measurement);
      estimator={
        ...estimator,
        measurement,
        estimate,
        controllerState:{
          x:estimate.x,
          z:estimate.z,
          theta:estimate.theta,
          vx:estimate.vx,
          vz:estimate.vz,
          q:estimate.q,
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
      predictiveEnabled,
      predictiveAverageSolveMs:predictiveEnabled?predictiveSolveMsSum/n:null,
      predictiveMaxSolveMs:predictiveEnabled?predictiveSolveMsMax:null,
      predictiveFeasibilityRate:predictiveEnabled?100*predictiveFeasibleCount/n:null,
      predictiveFallbackCount:predictiveEnabled?predictiveFallbackCount:0,
      predictiveInterventions:predictiveEnabled?predictiveInterventions:0,
      predictedMaxXError:predictiveEnabled?predictedMaxXError:null,
      predictedMaxZError:predictiveEnabled?predictedMaxZError:null,
      predictedMaxTilt:predictiveEnabled?predictedMaxTilt:null,
      predictedMaxVx:predictiveEnabled?predictedMaxVx:null,
      predictedMaxVz:predictiveEnabled?predictedMaxVz:null,
    },
  };
}
