import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createSecondOrderModel, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function rmse(values) {
  return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / Math.max(values.length, 1));
}

const cfg = {
  dt: 0.02,
  plant: { stiffness: 1.45, gain: 1.0, damping: 0.82 },
};
const model = createSecondOrderModel(cfg);
const sensor = createMeasurementSensor({ C: model.C, noiseStd: 0.035, seed: 20260914, bias: 0 });
const filter = createAugmentedDisturbanceKalmanFilter({
  A: model.A,
  B: model.B,
  E: model.E,
  C: model.C,
  processCovariance: [2e-5, 2e-4, 8e-3],
  measurementVariance: 0.035 ** 2,
  initialState: [0, 0, 0],
  initialCovariance: [0.04, 0.08, 0.8],
});

let state = { x: 0, v: 0 };
let measurement = sensor.read(state);
let estimate = filter.update(measurement.value);
const dErrors = [];
const xErrors = [];
const vErrors = [];
const activeEstimates = [];
const quietEstimates = [];

for (let k = 0; k < 420; k += 1) {
  const t = k * cfg.dt;
  const u = 0.35 * Math.sin(0.7 * t);
  const disturbance = t >= 2.0 && t <= 5.5 ? 1.15 : 0;
  state = stepSecondOrderPlant(state, u, disturbance, cfg);
  measurement = sensor.read(state);
  estimate = filter.step(u, measurement.value);

  xErrors.push(estimate.x - state.x);
  vErrors.push(estimate.v - state.v);
  if (t >= 3.0 && t <= 5.2) {
    dErrors.push(estimate.d - disturbance);
    activeEstimates.push(estimate.d);
  }
  if (t >= 7.0) quietEstimates.push(estimate.d);
}

const dRmse = rmse(dErrors);
const xRmse = rmse(xErrors);
const vRmse = rmse(vErrors);
const activeMean = activeEstimates.reduce((sum, value) => sum + value, 0) / activeEstimates.length;
const quietMeanAbs = quietEstimates.reduce((sum, value) => sum + Math.abs(value), 0) / quietEstimates.length;
const covariance = filter.getCovariance();

assert(Number.isFinite(dRmse) && dRmse < 0.38, `Disturbance RMSE too high: ${dRmse}`);
assert(activeMean > 0.72 && activeMean < 1.5, `Disturbance estimate mean is implausible: ${activeMean}`);
assert(quietMeanAbs < 0.45, `Disturbance estimate did not relax after pulse: ${quietMeanAbs}`);
assert(Number.isFinite(xRmse) && xRmse < 0.08, `Position RMSE too high: ${xRmse}`);
assert(Number.isFinite(vRmse) && vRmse < 0.3, `Velocity RMSE too high: ${vRmse}`);
assert(covariance.flat().every(Number.isFinite), 'Augmented covariance contains non-finite values.');
assert(covariance[0][0] >= 0 && covariance[1][1] >= 0 && covariance[2][2] >= 0, 'Augmented covariance diagonal became negative.');

console.log('Augmented disturbance-state Kalman smoke PASS');
console.log(`DisturbanceEstimation: dRMSE=${dRmse.toFixed(4)}, activeMean=${activeMean.toFixed(4)}, quietMeanAbs=${quietMeanAbs.toFixed(4)}, xRMSE=${xRmse.toFixed(4)}, vRMSE=${vRmse.toFixed(4)}`);
