import assert from 'node:assert/strict';

import { runPlanarUavClassicalBaseline } from '../src/core/orchestration/planarUavSimulator.js';

function scenario(predictiveEnabled,overrides={}){
  return {
    ...overrides,
    estimation:{
      enabled:true,
      seed:20260928,
      noiseStd:{x:0.08,z:0.06,theta:0.015,vx:0.08,vz:0.07,q:0.04},
      processCovariance:[1e-5,1e-5,1e-6,3e-4,3e-4,4e-4],
      initialCovariance:[0.12,0.10,0.03,0.15,0.15,0.08],
      ...(overrides.estimation||{}),
    },
    predictive:{
      enabled:predictiveEnabled,
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
      ...(overrides.predictive||{}),
    },
    reference:{...(overrides.reference||{})},
    safety:{...(overrides.safety||{})},
  };
}

function compact(result){
  return result.samples.map(s=>[
    s.t,s.x,s.z,s.theta,s.vx,s.vz,s.q,
    s.controllerX,s.controllerZ,s.controllerTheta,s.controllerVx,s.controllerVz,s.controllerQ,
    s.proposalThrust,s.proposalTorque,s.thrust,s.torque,
    s.predictiveFeasible,s.predictiveFallbackUsed,
    s.predictiveThrustOffset,s.predictiveTorqueOffset,
  ]);
}

function assertPhysical(result,label){
  const p=result.config.plant;
  assert.equal(result.metrics.unsafeSamples,0,`${label}: actual unsafe samples=${result.metrics.unsafeSamples}`);
  for(const sample of result.samples){
    assert(sample.thrust>=p.minThrust-1e-9 && sample.thrust<=p.maxThrust+1e-9,`${label}: thrust bound exceeded at t=${sample.t}`);
    assert(Math.abs(sample.torque)<=p.maxTorque+1e-9,`${label}: torque bound exceeded at t=${sample.t}`);
  }
}

const baseline=runPlanarUavClassicalBaseline(scenario(false));
const predictive=runPlanarUavClassicalBaseline(scenario(true));
const predictiveRepeat=runPlanarUavClassicalBaseline(scenario(true));

assert.deepEqual(compact(predictive),compact(predictiveRepeat),'predictive UAV run must be deterministic for same seed/config');
assertPhysical(baseline,'classical');
assertPhysical(predictive,'predictive');

const b=baseline.metrics;
const p=predictive.metrics;
console.log('Gate 5F probe',JSON.stringify({
  classical:{xRmse:b.xRmse,zRmse:b.zRmse,vxRmse:b.vxRmse,vzRmse:b.vzRmse,attitudeTrackingRmse:b.attitudeTrackingRmse,controlEffort:b.controlEffort,computePerStepUs:b.computePerStepUs},
  predictive:{xRmse:p.xRmse,zRmse:p.zRmse,vxRmse:p.vxRmse,vzRmse:p.vzRmse,attitudeTrackingRmse:p.attitudeTrackingRmse,controlEffort:p.controlEffort,computePerStepUs:p.computePerStepUs,feasibilityRate:p.predictiveFeasibilityRate,fallbackCount:p.predictiveFallbackCount,interventions:p.predictiveInterventions}
},null,2));
assert.equal(p.predictiveEnabled,true);
assert(p.predictiveInterventions>0,'predictive governor never changed the classical proposal');
assert.equal(p.predictiveFallbackCount,0,`predictive fallback count=${p.predictiveFallbackCount}`);
assert(p.predictiveFeasibilityRate>=99.9,`predicted feasibility rate too low: ${p.predictiveFeasibilityRate}`);
assert(p.predictedMaxXError<=predictive.config.safety.maxAbsXError+1e-9,`predicted x constraint exceeded: ${p.predictedMaxXError}`);
assert(p.predictedMaxZError<=predictive.config.safety.maxAbsZError+1e-9,`predicted z constraint exceeded: ${p.predictedMaxZError}`);
assert(p.predictedMaxTilt<=predictive.config.safety.maxAbsTilt+1e-9,`predicted tilt constraint exceeded: ${p.predictedMaxTilt}`);
assert(p.predictedMaxVx<=predictive.config.safety.maxAbsVx+1e-9,`predicted vx constraint exceeded: ${p.predictedMaxVx}`);
assert(p.predictedMaxVz<=predictive.config.safety.maxAbsVz+1e-9,`predicted vz constraint exceeded: ${p.predictedMaxVz}`);
assert(Number.isFinite(p.predictiveAverageSolveMs) && p.predictiveAverageSolveMs>0,'predictive solve timing missing');
assert(Number.isFinite(p.predictiveMaxSolveMs) && p.predictiveMaxSolveMs>=p.predictiveAverageSolveMs,'predictive max timing invalid');

assert(p.xRmse<=b.xRmse*1.10,`predictive x RMSE degraded >10%: ${b.xRmse} -> ${p.xRmse}`);
assert(p.zRmse<=b.zRmse*1.12,`predictive z RMSE degraded >12%: ${b.zRmse} -> ${p.zRmse}`);
assert(p.vxRmse<=b.vxRmse*1.12,`predictive vx RMSE degraded >12%: ${b.vxRmse} -> ${p.vxRmse}`);
assert(p.vzRmse<=b.vzRmse*1.15,`predictive vz RMSE degraded >15%: ${b.vzRmse} -> ${p.vzRmse}`);
assert(p.attitudeTrackingRmse<=b.attitudeTrackingRmse*1.15,`predictive attitude tracking degraded >15%: ${b.attitudeTrackingRmse} -> ${p.attitudeTrackingRmse}`);
assert(Number.isFinite(p.computePerStepUs) && p.computePerStepUs>0,'predictive compute timing missing');
assert(p.computePerStepUs<5000,`predictive simulation compute unexpectedly high: ${p.computePerStepUs} us/step`);

console.log('Gate 5F UAV planar constrained predictive comparison PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5f-uav-predictive/v1',
  classical:{
    xRmse:b.xRmse,
    zRmse:b.zRmse,
    vxRmse:b.vxRmse,
    vzRmse:b.vzRmse,
    attitudeTrackingRmse:b.attitudeTrackingRmse,
    controlEffort:b.controlEffort,
    unsafeSamples:b.unsafeSamples,
    computePerStepUs:b.computePerStepUs,
  },
  predictive:{
    xRmse:p.xRmse,
    zRmse:p.zRmse,
    vxRmse:p.vxRmse,
    vzRmse:p.vzRmse,
    attitudeTrackingRmse:p.attitudeTrackingRmse,
    controlEffort:p.controlEffort,
    unsafeSamples:p.unsafeSamples,
    computePerStepUs:p.computePerStepUs,
    averageSolveMs:p.predictiveAverageSolveMs,
    maxSolveMs:p.predictiveMaxSolveMs,
    feasibilityRate:p.predictiveFeasibilityRate,
    fallbackCount:p.predictiveFallbackCount,
    interventions:p.predictiveInterventions,
    predictedMaxXError:p.predictedMaxXError,
    predictedMaxZError:p.predictedMaxZError,
    predictedMaxTilt:p.predictedMaxTilt,
    predictedMaxVx:p.predictedMaxVx,
    predictedMaxVz:p.predictedMaxVz,
  },
  ratios:{
    x:p.xRmse/b.xRmse,
    z:p.zRmse/b.zRmse,
    vx:p.vxRmse/b.vxRmse,
    vz:p.vzRmse/b.vzRmse,
    attitude:p.attitudeTrackingRmse/b.attitudeTrackingRmse,
    effort:p.controlEffort/b.controlEffort,
    compute:p.computePerStepUs/b.computePerStepUs,
  },
},null,2));
