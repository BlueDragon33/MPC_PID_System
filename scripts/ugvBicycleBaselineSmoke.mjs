import assert from 'node:assert/strict';

import { createBicycleConfig, stepKinematicBicycle, wrapAngle } from '../src/core/models/kinematicBicycle.js';
import { defaultUGVScenario, runUGVBicycleBaseline } from '../src/core/orchestration/ugvBicycleSimulator.js';

const p=createBicycleConfig({dt:0.02,wheelbase:2.7,maxSteer:0.55,maxSteerRate:0.9,minSpeed:0,maxSpeed:8,minAccel:-3,maxAccel:2.5});
assert.equal(p.wheelbase,2.7);
assert(Math.abs(wrapAngle(3*Math.PI)-Math.PI)<1e-12);

const straight=stepKinematicBicycle({x:0,y:0,yaw:0,v:2},{steer:0,accel:0},p);
assert(Math.abs(straight.x-0.04)<1e-12);
assert(Math.abs(straight.y)<1e-12);
assert(Math.abs(straight.yaw)<1e-12);

const left=stepKinematicBicycle({x:0,y:0,yaw:0,v:2},{steer:0.25,accel:0},p);
assert(left.yaw>0,'positive steer must increase yaw');

const runA=runUGVBicycleBaseline();
const runB=runUGVBicycleBaseline();

const compact=(result)=>result.samples.map(s=>[
  s.t,s.x,s.y,s.yaw,s.v,s.cte,s.headingError,s.steer,s.accel,s.safetyViolation
]);
assert.deepEqual(compact(runA),compact(runB),'UGV baseline must be deterministic');

const m=runA.metrics;
assert(Number.isFinite(m.crossTrackRmse));
assert(Number.isFinite(m.headingRmse));
assert(Number.isFinite(m.speedRmse));
assert(Number.isFinite(m.computePerStepUs));
assert(m.crossTrackRmse<0.55,`cross-track RMSE too high: ${m.crossTrackRmse}`);
assert(m.headingRmse<0.22,`heading RMSE too high: ${m.headingRmse}`);
assert(m.speedRmse<0.95,`speed RMSE too high: ${m.speedRmse}`);
assert(m.maxAbsCrossTrack<=defaultUGVScenario.safety.maxAbsCrossTrack,`corridor exceeded: ${m.maxAbsCrossTrack}`);
assert(m.maxAbsHeadingError<=defaultUGVScenario.safety.maxAbsHeadingError,`heading safety exceeded: ${m.maxAbsHeadingError}`);
assert(m.maxSteerRate<=defaultUGVScenario.plant.maxSteerRate+1e-9,`steer-rate limit exceeded: ${m.maxSteerRate}`);
assert.equal(m.unsafeSamples,0,`unsafe samples: ${m.unsafeSamples}`);
assert(m.finalSpeed>3.7 && m.finalSpeed<4.3,`final speed implausible: ${m.finalSpeed}`);
assert(m.finalX>60,`vehicle did not progress far enough: ${m.finalX}`);
assert(m.computePerStepUs<500,`baseline simulation compute unexpectedly high: ${m.computePerStepUs} us/step`);

console.log('Gate 5A UGV bicycle baseline smoke PASS');
console.log(JSON.stringify({
  schema:'mpc-pid-gate5a-ugv-baseline/v1',
  metrics:m,
  scenario:{
    dt:runA.config.dt,
    duration:runA.config.duration,
    targetSpeed:runA.config.targetSpeed,
    wheelbase:runA.config.plant.wheelbase,
    path:runA.config.path,
  }
},null,2));
