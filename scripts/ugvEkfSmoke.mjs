import assert from 'node:assert/strict';

import { createUgvExtendedKalmanFilter } from '../src/core/estimation/ugvExtendedKalmanFilter.js';
import { createUgvInitialState, stepKinematicBicycle } from '../src/core/models/kinematicBicycle.js';
import { mergeUgvConfig } from '../src/core/orchestration/ugvSimulationConfig.js';

const cfg = mergeUgvConfig();
let truthA = createUgvInitialState({ y: 0.25, v: 2 });
let truthB = { ...truthA };
const filterA = createUgvExtendedKalmanFilter(cfg, createUgvInitialState({ y: 0.1, v: 1.8 }));
const filterB = createUgvExtendedKalmanFilter(cfg, createUgvInitialState({ y: 0.1, v: 1.8 }));
let a = filterA.initialize(truthA);
let b = filterB.initialize(truthB);
assert.deepEqual(a, b, 'same EKF seed must reproduce the initial update');

const errors = { x: [], y: [], yaw: [], v: [] };
for (let k = 0; k < 180; k += 1) {
  const command = {
    acceleration: 0.35 * Math.sin(0.04 * k),
    steering: 0.12 * Math.sin(0.025 * k),
  };
  truthA = stepKinematicBicycle(truthA, command, cfg);
  truthB = stepKinematicBicycle(truthB, command, cfg);
  a = filterA.step(command, truthA);
  b = filterB.step(command, truthB);
  assert.deepEqual(a, b, `same-seed EKF diverged at sample ${k}`);

  errors.x.push(a.state.x - truthA.x);
  errors.y.push(a.state.y - truthA.y);
  errors.yaw.push(a.state.yaw - truthA.yaw);
  errors.v.push(a.state.v - truthA.v);
}

const rmse = (values) => Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length);
const metrics = Object.fromEntries(Object.entries(errors).map(([key, values]) => [key, rmse(values)]));
assert(metrics.x < 0.08, `EKF x RMSE too high: ${metrics.x}`);
assert(metrics.y < 0.08, `EKF y RMSE too high: ${metrics.y}`);
assert(metrics.yaw < 0.025, `EKF yaw RMSE too high: ${metrics.yaw}`);
assert(metrics.v < 0.12, `EKF speed RMSE too high: ${metrics.v}`);
assert(a.covariance.flat().every(Number.isFinite));
assert(a.covariance.every((row, i) => row[i] >= 0));

console.log('UGV EKF smoke PASS', metrics);
