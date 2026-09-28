import assert from 'node:assert/strict';

import { runUGVBicycleBaseline } from '../src/core/orchestration/ugvBicycleSimulator.js';

function scenario(predictiveEnabled, overrides={}) {
  return {
    ...overrides,
    estimation:{
      enabled:true,
      seed:20260928,
      noiseStd:{x:0.18,y:0.18,yaw:0.035,v:0.12},
      processCovariance:[2e-4,2e-4,2e-5,3e-4],
      initialCovariance:[0.3,0.3,0.08,0.2],
      ...(overrides.estimation||{}),
    },
    predictive:{
      enabled:predictiveEnabled,
      horizon:10,
      predictionDt:0.08,
      qCrossTrack:8,
      qHeading:3,
      qSpeed:0.7,
      rSteer:0.12,
      rAccel:0.03,
      rProposal:0.22,
      ...(overrides.predictive||{}),
    },
    path:{...(overrides.path||{})},
    safety:{...(overrides.safety||{})},
  };
}

function compact(result){
  return result.samples.map(s=>[
    s.t,s.x,s.y,s.yaw,s.v,
    s.controllerX,s.controllerY,s.controllerYaw,s.controllerV,
    s.proposalSteer,s.proposalAccel,s.steer,s.accel,
    s.predictiveFeasible,s.predictiveFallbackUsed,
  ]);
}

function assertPhysical(result,label){
  const p=result.config.plant;
  assert.equal(result.metrics.unsafeSamples,0,`${label}: actual unsafe samples=${result.metrics.unsafeSamples}`);
  assert(result.metrics.maxSteerRate<=p.maxSteerRate+1e-9,`${label}: steering-rate bound exceeded: ${result.metrics.maxSteerRate}`);
  for(const sample of result.samples){
    assert(Math.abs(sample.steer)<=p.maxSteer+1e-9,`${label}: steer bound exceeded at t=${sample.t}`);
    assert(sample.accel>=p.minAccel-1e-9 && sample.accel<=p.maxAccel+1e-9,`${label}: accel bound exceeded at t=${sample.t}`);
    assert(sample.v>=p.minSpeed-1e-9 && sample.v<=p.maxSpeed+1e-9,`${label}: speed bound exceeded at t=${sample.t}`);
  }
}

const baseline=runUGVBicycleBaseline(scenario(false));
const predictive=runUGVBicycleBaseline(scenario(true));
const predictiveRepeat=runUGVBicycleBaseline(scenario(true));

assert.deepEqual(compact(predictive),compact(predictiveRepeat),'predictive UGV run must be deterministic for same seed/config');
assertPhysical(baseline,'classical');
assertPhysical(predictive,'predictive');

const b=baseline.metrics;
const p=predictive.metrics;
assert.equal(p.predictiveEnabled,true);
assert(p.predictiveInterventions>0,'predictive governor never changed the classical proposal');
assert.equal(p.predictiveFallbackCount,0,`predictive fallback count=${p.predictiveFallbackCount}`);
assert(p.predictiveFeasibilityRate>=99.9,`predicted feasibility rate too low: ${p.predictiveFeasibilityRate}`);
assert(p.predictedMaxCrossTrack<=predictive.config.safety.maxAbsCrossTrack+1e-9,`predicted cross-track constraint exceeded: ${p.predictedMaxCrossTrack}`);
assert(p.predictedMaxHeadingError<=predictive.config.safety.maxAbsHeadingError+1e-9,`predicted heading constraint exceeded: ${p.predictedMaxHeadingError}`);
assert(Number.isFinite(p.predictiveAverageSolveMs) && p.predictiveAverageSolveMs>0,'predictive solve timing missing');
assert(Number.isFinite(p.predictiveMaxSolveMs) && p.predictiveMaxSolveMs>=p.predictiveAverageSolveMs,'predictive max timing invalid');

assert(p.crossTrackRmse<=b.crossTrackRmse*1.05,`predictive cross-track RMSE degraded >5%: ${b.crossTrackRmse} -> ${p.crossTrackRmse}`);
assert(p.headingRmse<=b.headingRmse*1.10,`predictive heading RMSE degraded >10%: ${b.headingRmse} -> ${p.headingRmse}`);
assert(p.speedRmse<=b.speedRmse*1.08,`predictive speed RMSE degraded >8%: ${b.speedRmse} -> ${p.speedRmse}`);
assert(p.computePerStepUs>b.computePerStepUs,`predictive compute should expose measurable overhead: ${b.computePerStepUs} -> ${p.computePerStepUs}`);
assert(p.computePerStepUs<1500,`predictive simulation compute unexpectedly high: ${p.computePerStepUs} us/step`);

console.log('Gate 5C UGV constrained predictive comparison PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5c-predictive-governor/v1',
  classical:{
    crossTrackRmse:b.crossTrackRmse,
    headingRmse:b.headingRmse,
    speedRmse:b.speedRmse,
    controlEffort:b.controlEffort,
    unsafeSamples:b.unsafeSamples,
    computePerStepUs:b.computePerStepUs,
  },
  predictive:{
    crossTrackRmse:p.crossTrackRmse,
    headingRmse:p.headingRmse,
    speedRmse:p.speedRmse,
    controlEffort:p.controlEffort,
    unsafeSamples:p.unsafeSamples,
    computePerStepUs:p.computePerStepUs,
    averageSolveMs:p.predictiveAverageSolveMs,
    maxSolveMs:p.predictiveMaxSolveMs,
    feasibilityRate:p.predictiveFeasibilityRate,
    fallbackCount:p.predictiveFallbackCount,
    interventions:p.predictiveInterventions,
    predictedMaxCrossTrack:p.predictedMaxCrossTrack,
    predictedMaxHeadingError:p.predictedMaxHeadingError,
  },
  ratios:{
    crossTrack:p.crossTrackRmse/b.crossTrackRmse,
    heading:p.headingRmse/b.headingRmse,
    speed:p.speedRmse/b.speedRmse,
    compute:p.computePerStepUs/b.computePerStepUs,
  }
},null,2));
