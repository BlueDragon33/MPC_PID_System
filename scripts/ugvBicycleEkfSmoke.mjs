import assert from 'node:assert/strict';

import { runUGVBicycleBaseline } from '../src/core/orchestration/ugvBicycleSimulator.js';

function estimatedScenario(seed=20260928){
  return {
    estimation:{
      enabled:true,
      seed,
      noiseStd:{x:0.18,y:0.18,yaw:0.035,v:0.12},
      processCovariance:[2e-4,2e-4,2e-5,3e-4],
      initialCovariance:[0.3,0.3,0.08,0.2],
    },
  };
}

const truth=runUGVBicycleBaseline();
const estimatedA=runUGVBicycleBaseline(estimatedScenario(20260928));
const estimatedB=runUGVBicycleBaseline(estimatedScenario(20260928));
const estimatedOtherSeed=runUGVBicycleBaseline(estimatedScenario(20260929));

const trace=result=>result.samples.map(s=>[
  s.measurementX,s.measurementY,s.measurementYaw,s.measurementV,
  s.controllerX,s.controllerY,s.controllerYaw,s.controllerV,
  s.steer,s.accel
]);

assert.deepEqual(trace(estimatedA),trace(estimatedB),'same seed must reproduce UGV measurement/estimate/command trace exactly');
assert.notDeepEqual(trace(estimatedA),trace(estimatedOtherSeed),'different seed must change noisy closed-loop trace');

assert.equal(estimatedA.metrics.estimationEnabled,true);
assert(estimatedA.samples.some(s=>
  Math.abs(s.controllerX-s.x)>1e-9 ||
  Math.abs(s.controllerY-s.y)>1e-9 ||
  Math.abs(s.controllerYaw-s.yaw)>1e-9 ||
  Math.abs(s.controllerV-s.v)>1e-9
),'controller state unexpectedly equals ground truth at every sample');

const m=estimatedA.metrics;
for(const key of ['x','y','yaw','v']){
  assert(Number.isFinite(m.measurementRmse[key]),`measurement RMSE ${key} is not finite`);
  assert(Number.isFinite(m.estimateRmse[key]),`estimate RMSE ${key} is not finite`);
  assert(m.estimateRmse[key] < m.measurementRmse[key],`EKF failed to improve ${key}: measurement=${m.measurementRmse[key]}, estimate=${m.estimateRmse[key]}`);
}
assert(Number.isFinite(m.averageCovarianceTrace) && m.averageCovarianceTrace>0,'covariance trace is invalid');
assert.equal(m.unsafeSamples,0,`estimated-state loop produced ${m.unsafeSamples} unsafe samples`);
assert(m.crossTrackRmse <= truth.metrics.crossTrackRmse*1.35,`estimated-state cross-track RMSE degraded excessively: ${truth.metrics.crossTrackRmse} -> ${m.crossTrackRmse}`);
assert(m.headingRmse <= truth.metrics.headingRmse*1.5,`estimated-state heading RMSE degraded excessively: ${truth.metrics.headingRmse} -> ${m.headingRmse}`);
assert(m.speedRmse <= truth.metrics.speedRmse*1.25,`estimated-state speed RMSE degraded excessively: ${truth.metrics.speedRmse} -> ${m.speedRmse}`);
assert(m.maxSteerRate <= estimatedA.config.plant.maxSteerRate+1e-9,`steering rate exceeded under estimation: ${m.maxSteerRate}`);

console.log('Gate 5B UGV nonlinear EKF closed-loop smoke PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5b-ugv-ekf/v1',
  truth:{
    crossTrackRmse:truth.metrics.crossTrackRmse,
    headingRmse:truth.metrics.headingRmse,
    speedRmse:truth.metrics.speedRmse,
  },
  estimated:{
    crossTrackRmse:m.crossTrackRmse,
    headingRmse:m.headingRmse,
    speedRmse:m.speedRmse,
    measurementRmse:m.measurementRmse,
    estimateRmse:m.estimateRmse,
    averageCovarianceTrace:m.averageCovarianceTrace,
    unsafeSamples:m.unsafeSamples,
    computePerStepUs:m.computePerStepUs,
  }
},null,2));
