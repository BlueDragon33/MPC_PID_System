import assert from 'node:assert/strict';

import {
  createPlanarUavConfig,
  planarUavDerivatives,
  stepPlanarUav,
} from '../src/core/models/planarUav.js';
import {
  defaultPlanarUavScenario,
  runPlanarUavClassicalBaseline,
} from '../src/core/orchestration/planarUavSimulator.js';

const plant=createPlanarUavConfig({
  ...defaultPlanarUavScenario.plant,
  dt:defaultPlanarUavScenario.dt,
});
const hoverState={x:0,z:2,theta:0,vx:0,vz:0,q:0};
const hoverInput={thrust:plant.mass*plant.gravity,torque:0};
const hoverDerivative=planarUavDerivatives(hoverState,hoverInput,plant);
assert(Math.abs(hoverDerivative.vxDot)<1e-12,'hover horizontal acceleration must be zero');
assert(Math.abs(hoverDerivative.vzDot)<1e-12,'hover vertical acceleration must be zero');
assert(Math.abs(hoverDerivative.qDot)<1e-12,'hover angular acceleration must be zero');

const tiltedDerivative=planarUavDerivatives(
  {...hoverState,theta:0.2},
  hoverInput,
  plant,
);
assert(tiltedDerivative.vxDot<0,'positive pitch must accelerate toward negative x under this sign convention');
assert(tiltedDerivative.vzDot<0,'tilted hover thrust must lose vertical authority');

const torqueStep=stepPlanarUav(
  hoverState,
  {thrust:hoverInput.thrust,torque:0.25},
  plant,
);
assert(torqueStep.q>0,'positive torque must increase pitch rate');

const runA=runPlanarUavClassicalBaseline();
const runB=runPlanarUavClassicalBaseline();
const trace=(result)=>result.samples.map((s)=>[
  s.t,s.x,s.z,s.theta,s.vx,s.vz,s.q,
  s.refX,s.refZ,s.desiredTheta,s.thrust,s.torque,s.safetyViolation,
]);
assert.deepEqual(trace(runA),trace(runB),'planar UAV baseline must be deterministic');

const m=runA.metrics;
for(const key of [
  'xRmse','zRmse','vxRmse','vzRmse','attitudeTrackingRmse',
  'maxAbsXError','maxAbsZError','maxAbsTilt','controlEffort','computePerStepUs',
]){
  assert(Number.isFinite(m[key]),`${key} must be finite`);
}

assert(m.xRmse<0.32,`x RMSE too high: ${m.xRmse}`);
assert(m.zRmse<0.14,`z RMSE too high: ${m.zRmse}`);
assert(m.vxRmse<0.32,`vx RMSE too high: ${m.vxRmse}`);
assert(m.vzRmse<0.22,`vz RMSE too high: ${m.vzRmse}`);
assert(m.attitudeTrackingRmse<0.04,`attitude tracking RMSE too high: ${m.attitudeTrackingRmse}`);
assert(m.maxAbsXError<=runA.config.safety.maxAbsXError+1e-9,`x safety corridor exceeded: ${m.maxAbsXError}`);
assert(m.maxAbsZError<=runA.config.safety.maxAbsZError+1e-9,`z safety corridor exceeded: ${m.maxAbsZError}`);
assert(m.maxAbsTilt<=runA.config.safety.maxAbsTilt+1e-9,`tilt safety exceeded: ${m.maxAbsTilt}`);
assert(m.maxThrust<=runA.config.plant.maxThrust+1e-9,`thrust bound exceeded: ${m.maxThrust}`);
assert(m.maxTorque<=runA.config.plant.maxTorque+1e-9,`torque bound exceeded: ${m.maxTorque}`);
assert.equal(m.unsafeSamples,0,`unsafe samples: ${m.unsafeSamples}`);
assert(m.finalVx>1.0 && m.finalVx<1.2,`final vx implausible: ${m.finalVx}`);
assert(m.finalZ>1.7 && m.finalZ<2.2,`final altitude implausible: ${m.finalZ}`);
assert(m.finalX>17,`UAV did not progress far enough: ${m.finalX}`);
assert(m.computePerStepUs<500,`baseline simulation compute unexpectedly high: ${m.computePerStepUs} us/step`);

console.log('Gate 5D UAV planar classical baseline smoke PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5d-uav-planar-baseline/v1',
  metrics:m,
  scenario:{
    dt:runA.config.dt,
    duration:runA.config.duration,
    reference:runA.config.reference,
    plant:runA.config.plant,
    safety:runA.config.safety,
  },
},null,2));
