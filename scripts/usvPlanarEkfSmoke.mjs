import assert from 'node:assert/strict';

import { runPlanarUsvClassicalBaseline } from '../src/core/orchestration/planarUsvSimulator.js';

function estimatedScenario(seed=20260929){
  return {
    estimation:{
      enabled:true,
      seed,
      noiseStd:{x:0.12,y:0.12,psi:0.02,u:0.08,v:0.05,r:0.025},
      processCovariance:[1e-5,1e-5,1e-6,2e-4,2e-4,1e-4],
      initialCovariance:[0.2,0.2,0.04,0.15,0.08,0.05],
    },
  };
}

const truth=runPlanarUsvClassicalBaseline();
const estimatedA=runPlanarUsvClassicalBaseline(estimatedScenario(20260929));
const estimatedB=runPlanarUsvClassicalBaseline(estimatedScenario(20260929));
const estimatedOtherSeed=runPlanarUsvClassicalBaseline(estimatedScenario(20260930));

const trace=result=>result.samples.map(sample=>[
  sample.measurementX,
  sample.measurementY,
  sample.measurementPsi,
  sample.measurementU,
  sample.measurementV,
  sample.measurementR,
  sample.controllerX,
  sample.controllerY,
  sample.controllerPsi,
  sample.controllerU,
  sample.controllerV,
  sample.controllerR,
  sample.surgeForce,
  sample.yawMoment,
]);

assert.deepEqual(
  trace(estimatedA),
  trace(estimatedB),
  'same seed must reproduce USV measurement/estimate/command trace exactly',
);
assert.notDeepEqual(
  trace(estimatedA),
  trace(estimatedOtherSeed),
  'different seed must change noisy USV closed-loop trace',
);

assert.equal(estimatedA.metrics.estimationEnabled,true);
assert(
  estimatedA.samples.some(sample=>
    Math.abs(sample.controllerX-sample.x)>1e-9 ||
    Math.abs(sample.controllerY-sample.y)>1e-9 ||
    Math.abs(sample.controllerPsi-sample.psi)>1e-9 ||
    Math.abs(sample.controllerU-sample.u)>1e-9 ||
    Math.abs(sample.controllerV-sample.v)>1e-9 ||
    Math.abs(sample.controllerR-sample.r)>1e-9
  ),
  'controller state unexpectedly equals ground truth at every sample',
);

const m=estimatedA.metrics;
for(const key of ['x','y','psi','u','v','r']){
  assert(Number.isFinite(m.measurementRmse[key]),`measurement RMSE ${key} is not finite`);
  assert(Number.isFinite(m.estimateRmse[key]),`estimate RMSE ${key} is not finite`);
  assert(
    m.estimateRmse[key] < m.measurementRmse[key],
    `EKF failed to improve ${key}: measurement=${m.measurementRmse[key]}, estimate=${m.estimateRmse[key]}`,
  );
}

assert(
  Number.isFinite(m.averageCovarianceTrace) && m.averageCovarianceTrace>0,
  'covariance trace is invalid',
);
assert.equal(m.unsafeSamples,0,`estimated-state loop produced ${m.unsafeSamples} unsafe samples`);
assert(
  m.crossTrackRmse<=truth.metrics.crossTrackRmse*1.4,
  `estimated-state cross-track RMSE degraded excessively: ${truth.metrics.crossTrackRmse} -> ${m.crossTrackRmse}`,
);
assert(
  m.headingRmse<=truth.metrics.headingRmse*1.5,
  `estimated-state heading RMSE degraded excessively: ${truth.metrics.headingRmse} -> ${m.headingRmse}`,
);
assert(
  m.speedRmse<=truth.metrics.speedRmse*1.3,
  `estimated-state speed RMSE degraded excessively: ${truth.metrics.speedRmse} -> ${m.speedRmse}`,
);
assert(
  m.maxSurgeForce<=estimatedA.config.plant.maxSurgeForce+1e-9,
  `surge force bound exceeded under estimation: ${m.maxSurgeForce}`,
);
assert(
  m.maxYawMoment<=estimatedA.config.plant.maxYawMoment+1e-9,
  `yaw moment bound exceeded under estimation: ${m.maxYawMoment}`,
);
assert(
  m.maxSurgeForceRate<=estimatedA.config.actuator.maxSurgeForceRate+1e-9,
  `surge force rate exceeded under estimation: ${m.maxSurgeForceRate}`,
);
assert(
  m.maxYawMomentRate<=estimatedA.config.actuator.maxYawMomentRate+1e-9,
  `yaw moment rate exceeded under estimation: ${m.maxYawMomentRate}`,
);

console.log('Gate 5H USV planar nonlinear EKF closed-loop smoke PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5h-usv-ekf/v1',
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
    maxViolation:m.maxViolation,
    computePerStepUs:m.computePerStepUs,
  },
},null,2));
