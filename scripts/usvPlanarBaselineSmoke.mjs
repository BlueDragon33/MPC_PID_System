import assert from 'node:assert/strict';

import {
  createPlanarUsvConfig,
  planarUsvDerivatives,
  stepPlanarUsv,
} from '../src/core/models/planarUsv.js';
import {
  defaultPlanarUsvScenario,
  runPlanarUsvClassicalBaseline,
} from '../src/core/orchestration/planarUsvSimulator.js';

const plant=createPlanarUsvConfig({
  ...defaultPlanarUsvScenario.plant,
  dt:defaultPlanarUsvScenario.dt,
});
const rest={x:0,y:0,psi:0,u:0,v:0,r:0};
const idle=planarUsvDerivatives(rest,{surgeForce:0,yawMoment:0},plant);
assert(Math.abs(idle.uDot)<1e-12,'idle surge acceleration must be zero');
assert(Math.abs(idle.rDot)<1e-12,'idle yaw acceleration must be zero');

const forward=planarUsvDerivatives(rest,{surgeForce:20,yawMoment:0},plant);
assert(forward.uDot>0,'positive surge force must accelerate the vessel forward');

const yawStep=stepPlanarUsv(rest,{surgeForce:0,yawMoment:5},plant);
assert(yawStep.r>0,'positive yaw moment must increase yaw rate');

const runA=runPlanarUsvClassicalBaseline();
const runB=runPlanarUsvClassicalBaseline();
const trace=(result)=>result.samples.map((sample)=>[
  sample.t,
  sample.x,
  sample.y,
  sample.psi,
  sample.u,
  sample.v,
  sample.r,
  sample.refY,
  sample.refHeading,
  sample.surgeForce,
  sample.yawMoment,
  sample.safetyViolation,
]);
assert.deepEqual(trace(runA),trace(runB),'USV baseline must be deterministic');

const m=runA.metrics;
for(const key of [
  'crossTrackRmse',
  'headingRmse',
  'speedRmse',
  'maxAbsCrossTrack',
  'maxAbsHeadingError',
  'maxSpeed',
  'maxAbsYawRate',
  'controlEffort',
  'computePerStepUs',
]){
  assert(Number.isFinite(m[key]),`${key} must be finite`);
}

assert(m.crossTrackRmse<0.65,`cross-track RMSE too high: ${m.crossTrackRmse}`);
assert(m.headingRmse<0.24,`heading RMSE too high: ${m.headingRmse}`);
assert(m.speedRmse<0.55,`speed RMSE too high: ${m.speedRmse}`);
assert(
  m.maxAbsCrossTrack<=runA.config.safety.maxAbsCrossTrack+1e-9,
  `cross-track safety corridor exceeded: ${m.maxAbsCrossTrack}`,
);
assert(
  m.maxAbsHeadingError<=runA.config.safety.maxAbsHeadingError+1e-9,
  `heading safety exceeded: ${m.maxAbsHeadingError}`,
);
assert(m.maxSpeed<=runA.config.safety.maxSpeed+1e-9,`speed safety exceeded: ${m.maxSpeed}`);
assert(
  m.maxAbsYawRate<=runA.config.safety.maxAbsYawRate+1e-9,
  `yaw-rate safety exceeded: ${m.maxAbsYawRate}`,
);
assert(
  m.maxSurgeForce<=runA.config.plant.maxSurgeForce+1e-9,
  `surge-force bound exceeded: ${m.maxSurgeForce}`,
);
assert(
  m.maxYawMoment<=runA.config.plant.maxYawMoment+1e-9,
  `yaw-moment bound exceeded: ${m.maxYawMoment}`,
);
assert(
  m.maxSurgeForceRate<=runA.config.actuator.maxSurgeForceRate+1e-9,
  `surge-force rate exceeded: ${m.maxSurgeForceRate}`,
);
assert(
  m.maxYawMomentRate<=runA.config.actuator.maxYawMomentRate+1e-9,
  `yaw-moment rate exceeded: ${m.maxYawMomentRate}`,
);
assert.equal(m.unsafeSamples,0,`unsafe samples: ${m.unsafeSamples}`);
assert(
  m.finalSurgeSpeed>1.65 && m.finalSurgeSpeed<1.95,
  `final surge speed implausible: ${m.finalSurgeSpeed}`,
);
assert(m.finalX>45,`USV did not progress far enough: ${m.finalX}`);
assert(
  m.computePerStepUs<500,
  `baseline simulation compute unexpectedly high: ${m.computePerStepUs} us/step`,
);

console.log('Gate 5G USV planar classical baseline smoke PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5g-usv-planar-baseline/v1',
  metrics:m,
  scenario:{
    dt:runA.config.dt,
    duration:runA.config.duration,
    targetSpeed:runA.config.targetSpeed,
    path:runA.config.path,
    plant:runA.config.plant,
    actuator:runA.config.actuator,
    safety:runA.config.safety,
  },
},null,2));
