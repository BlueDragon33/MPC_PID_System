import { createUGVPathFollower } from '../controllers/ugvPathFollower.js';
import { createUGVPredictiveGovernor } from '../controllers/ugvPredictiveGovernor.js';
import { createUGVBicycleEkf } from '../estimation/ugvBicycleEkf.js';
import { createUGVMeasurementSensor } from '../estimation/ugvMeasurementSensor.js';
import { createBicycleConfig, stepKinematicBicycle, wrapAngle } from '../models/kinematicBicycle.js';

export const defaultUGVScenario={
  dt:0.02,duration:22,targetSpeed:4.0,
  path:{amplitude:1.2,wavelength:34},
  plant:{wheelbase:2.7,maxSteer:0.55,maxSteerRate:0.9,minSpeed:0,maxSpeed:8,minAccel:-3,maxAccel:2.5},
  controller:{stanleyGain:1.35,softening:0.8,speedKp:1.8,speedKi:0.35,speedKd:0.08},
  estimation:{
    enabled:false,
    seed:20260928,
    noiseStd:{x:0.18,y:0.18,yaw:0.035,v:0.12},
    processCovariance:[2e-4,2e-4,2e-5,3e-4],
    initialCovariance:[0.3,0.3,0.08,0.2],
  },
  predictive:{
    enabled:false,
    horizon:10,
    predictionDt:0.08,
    qCrossTrack:8,
    qHeading:3,
    qSpeed:0.7,
    rSteer:0.12,
    rAccel:0.03,
    rProposal:0.22,
  },
  safety:{maxAbsCrossTrack:1.25,maxAbsHeadingError:0.8}
};
const sq=x=>x*x;
function merge(cfg={}){
  return {
    ...defaultUGVScenario,...cfg,
    path:{...defaultUGVScenario.path,...(cfg.path||{})},
    plant:{...defaultUGVScenario.plant,...(cfg.plant||{})},
    controller:{...defaultUGVScenario.controller,...(cfg.controller||{})},
    estimation:{
      ...defaultUGVScenario.estimation,...(cfg.estimation||{}),
      noiseStd:{...defaultUGVScenario.estimation.noiseStd,...(cfg.estimation?.noiseStd||{})},
    },
    predictive:{...defaultUGVScenario.predictive,...(cfg.predictive||{})},
    safety:{...defaultUGVScenario.safety,...(cfg.safety||{})}
  };
}
export function ugvPathReference(x,cfg){
  const A=cfg.path.amplitude,k=2*Math.PI/cfg.path.wavelength;
  const y=A*Math.sin(k*x);
  const dydx=A*k*Math.cos(k*x);
  return {x,y,heading:Math.atan(dydx),speed:cfg.targetSpeed};
}
export function runUGVBicycleBaseline(userCfg={}){
  const cfg=merge(userCfg); createBicycleConfig({...cfg.plant,dt:cfg.dt});
  const controller=createUGVPathFollower({...cfg.controller,...cfg.plant,dt:cfg.dt});
  const predictiveEnabled=Boolean(cfg.predictive.enabled);
  const predictiveGovernor=predictiveEnabled?createUGVPredictiveGovernor({
    ...cfg.predictive,
    ...cfg.plant,
    ...cfg.safety,
    dt:cfg.dt,
  }):null;
  let previousCommand={steer:0,accel:0};
  let state={x:0,y:0.65,yaw:0,v:0};
  const estimationEnabled=Boolean(cfg.estimation.enabled);
  const sensor=estimationEnabled?createUGVMeasurementSensor({
    seed:cfg.estimation.seed,
    noiseStd:cfg.estimation.noiseStd,
  }):null;
  let measurement=estimationEnabled?sensor.read(state):null;
  const ekf=estimationEnabled?createUGVBicycleEkf({
    cfg:{...cfg.plant,dt:cfg.dt},
    initialState:[measurement.x,measurement.y,measurement.yaw,measurement.v],
    initialCovariance:cfg.estimation.initialCovariance,
    processCovariance:cfg.estimation.processCovariance,
    measurementVariance:[
      cfg.estimation.noiseStd.x**2,
      cfg.estimation.noiseStd.y**2,
      cfg.estimation.noiseStd.yaw**2,
      cfg.estimation.noiseStd.v**2,
    ],
  }):null;
  let estimate=estimationEnabled?ekf.update(measurement):{...state,covariance:null,diagnostics:null};
  let controllerState=estimationEnabled?{x:estimate.x,y:estimate.y,yaw:estimate.yaw,v:estimate.v}:{...state};

  let prevSteer=0; const samples=[]; const steps=Math.floor(cfg.duration/cfg.dt);
  let sumCte2=0,sumHeading2=0,sumSpeed2=0,sumEffort=0,maxCte=0,maxHeading=0,maxSteerRate=0,unsafe=0;
  let predictiveSolveMsSum=0,predictiveSolveMsMax=0,predictiveFeasibleCount=0,predictiveFallbackCount=0,predictiveInterventions=0;
  let predictedMaxCrossTrack=0,predictedMaxHeadingError=0;
  let measurementError2={x:0,y:0,yaw:0,v:0},estimateError2={x:0,y:0,yaw:0,v:0},covarianceTraceSum=0;
  const start=performance.now();
  for(let k=0;k<=steps;k++){
    const t=k*cfg.dt;
    const controlRef=ugvPathReference(controllerState.x,cfg);
    const proposal=controller.update(controllerState,controlRef);
    let predictiveDiagnostics=null;
    let cmd=proposal;
    if(predictiveEnabled){
      const selected=predictiveGovernor.select({
        state:controllerState,
        proposal,
        previousCommand,
        referenceAt:(x)=>ugvPathReference(x,cfg),
        plantCfg:cfg.plant,
      });
      cmd=selected.command;
      predictiveDiagnostics=selected.diagnostics;
      predictiveSolveMsSum+=predictiveDiagnostics.solveMs;
      predictiveSolveMsMax=Math.max(predictiveSolveMsMax,predictiveDiagnostics.solveMs);
      if(predictiveDiagnostics.predictedFeasible) predictiveFeasibleCount+=1;
      if(predictiveDiagnostics.fallbackUsed) predictiveFallbackCount+=1;
      if(Math.abs(cmd.steer-proposal.steer)>1e-12 || Math.abs(cmd.accel-proposal.accel)>1e-12) predictiveInterventions+=1;
      predictedMaxCrossTrack=Math.max(predictedMaxCrossTrack,predictiveDiagnostics.predictedMaxCrossTrack??0);
      predictedMaxHeadingError=Math.max(predictedMaxHeadingError,predictiveDiagnostics.predictedMaxHeadingError??0);
    }
    controller.syncAppliedSteer(cmd.steer);
    const steerRate=(cmd.steer-prevSteer)/cfg.dt; prevSteer=cmd.steer;
    previousCommand={...cmd};

    const auditRef=ugvPathReference(state.x,cfg);
    const headingError=wrapAngle(auditRef.heading-state.yaw);
    const cte=auditRef.y-state.y;
    const speedError=auditRef.speed-state.v;
    maxCte=Math.max(maxCte,Math.abs(cte)); maxHeading=Math.max(maxHeading,Math.abs(headingError)); maxSteerRate=Math.max(maxSteerRate,Math.abs(steerRate));
    sumCte2+=sq(cte);sumHeading2+=sq(headingError);sumSpeed2+=sq(speedError);sumEffort+=sq(cmd.steer)+0.12*sq(cmd.accel);
    const safetyViolation=Math.max(0,Math.abs(cte)-cfg.safety.maxAbsCrossTrack,Math.abs(headingError)-cfg.safety.maxAbsHeadingError);
    if(safetyViolation>0) unsafe++;

    if(estimationEnabled){
      measurementError2.x+=sq(measurement.x-state.x);
      measurementError2.y+=sq(measurement.y-state.y);
      measurementError2.yaw+=sq(wrapAngle(measurement.yaw-state.yaw));
      measurementError2.v+=sq(measurement.v-state.v);
      estimateError2.x+=sq(controllerState.x-state.x);
      estimateError2.y+=sq(controllerState.y-state.y);
      estimateError2.yaw+=sq(wrapAngle(controllerState.yaw-state.yaw));
      estimateError2.v+=sq(controllerState.v-state.v);
      covarianceTraceSum+=estimate.diagnostics?.covarianceTrace??0;
    }

    samples.push({
      t,...state,
      controllerX:controllerState.x,controllerY:controllerState.y,controllerYaw:controllerState.yaw,controllerV:controllerState.v,
      refY:auditRef.y,refHeading:auditRef.heading,refSpeed:auditRef.speed,
      cte,headingError,speedError,steer:cmd.steer,accel:cmd.accel,steerRate,safetyViolation,
      proposalSteer:proposal.steer,proposalAccel:proposal.accel,
      predictiveEnabled,
      predictiveFeasible:predictiveDiagnostics?.predictedFeasible??null,
      predictiveFallbackUsed:predictiveDiagnostics?.fallbackUsed??false,
      predictiveCost:predictiveDiagnostics?.cost??null,
      predictiveSolveMs:predictiveDiagnostics?.solveMs??null,
      predictedMaxCrossTrack:predictiveDiagnostics?.predictedMaxCrossTrack??null,
      predictedMaxHeadingError:predictiveDiagnostics?.predictedMaxHeadingError??null,
      predictiveCandidateCount:predictiveDiagnostics?.candidateCount??null,
      predictiveFeasibleCount:predictiveDiagnostics?.feasibleCount??null,
      estimationEnabled,
      measurementX:measurement?.x??null,measurementY:measurement?.y??null,measurementYaw:measurement?.yaw??null,measurementV:measurement?.v??null,
      covarianceTrace:estimationEnabled?(estimate.diagnostics?.covarianceTrace??null):null,
      innovations:estimationEnabled?(estimate.diagnostics?.innovations??null):null,
    });

    state=stepKinematicBicycle(state,cmd,{...cfg.plant,dt:cfg.dt});
    if(estimationEnabled){
      measurement=sensor.read(state);
      estimate=ekf.step(cmd,measurement);
      controllerState={x:estimate.x,y:estimate.y,yaw:estimate.yaw,v:estimate.v};
    }else{
      controllerState={...state};
    }
  }
  const elapsedMs=performance.now()-start,n=samples.length;
  const rmse=(errors)=>({
    x:Math.sqrt(errors.x/n),y:Math.sqrt(errors.y/n),yaw:Math.sqrt(errors.yaw/n),v:Math.sqrt(errors.v/n)
  });
  return {samples,config:cfg,metrics:{
    crossTrackRmse:Math.sqrt(sumCte2/n),headingRmse:Math.sqrt(sumHeading2/n),speedRmse:Math.sqrt(sumSpeed2/n),
    maxAbsCrossTrack:maxCte,maxAbsHeadingError:maxHeading,maxSteerRate,controlEffort:sumEffort*cfg.dt,
    unsafeSamples:unsafe,unsafeRate:100*unsafe/n,finalSpeed:state.v,finalX:state.x,
    simulationMs:elapsedMs,computePerStepUs:1000*elapsedMs/n,
    estimationEnabled,
    measurementRmse:estimationEnabled?rmse(measurementError2):null,
    estimateRmse:estimationEnabled?rmse(estimateError2):null,
    averageCovarianceTrace:estimationEnabled?covarianceTraceSum/n:null,
    predictiveEnabled,
    predictiveAverageSolveMs:predictiveEnabled?predictiveSolveMsSum/n:null,
    predictiveMaxSolveMs:predictiveEnabled?predictiveSolveMsMax:null,
    predictiveFeasibilityRate:predictiveEnabled?100*predictiveFeasibleCount/n:null,
    predictiveFallbackCount:predictiveEnabled?predictiveFallbackCount:0,
    predictiveInterventions:predictiveEnabled?predictiveInterventions:0,
    predictedMaxCrossTrack:predictiveEnabled?predictedMaxCrossTrack:null,
    predictedMaxHeadingError:predictiveEnabled?predictedMaxHeadingError:null,
  }};
}
