import { createSecondOrderModel, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createSeededGaussian } from '../src/core/estimation/deterministicNoise.js';
import { createLinearKalmanFilter } from '../src/core/estimation/linearKalmanFilter.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function rmse(values) {
  return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / Math.max(1, values.length));
}

const cfg = {
  dt: 0.02,
  plant: { stiffness: 1.45, gain: 1.0, damping: 0.82 },
};
const model = createSecondOrderModel(cfg);
const measurementStd = 0.12;
const gaussian = createSeededGaussian(20260914);
const filter = createLinearKalmanFilter({
  A: model.A,
  B: model.B,
  C: model.C,
  processCovariance: [2e-5, 2e-4],
  measurementVariance: measurementStd * measurementStd,
  initialState: [0, 0],
  initialCovariance: [0.25, 0.8],
});

let state = { x: 0, v: 0 };
const measurementErrors = [];
const estimatePositionErrors = [];
const estimateVelocityErrors = [];
const innovations = [];

for (let k = 0; k < 600; k += 1) {
  const u = 1.1 * Math.sin(k * 0.037) + (k > 180 ? 0.55 : -0.15) + (k > 390 ? -0.75 : 0);
  state = stepSecondOrderPlant(state, u, 0, cfg);
  const measurement = state.x + gaussian(0, measurementStd);
  const estimate = filter.step(u, measurement);

  measurementErrors.push(measurement - state.x);
  estimatePositionErrors.push(estimate.x - state.x);
  estimateVelocityErrors.push(estimate.v - state.v);
  innovations.push(estimate.diagnostics.innovation);

  assert(Number.isFinite(estimate.x) && Number.isFinite(estimate.v), `Non-finite estimate at sample ${k}`);
  assert(Number.isFinite(estimate.diagnostics.innovationVariance) && estimate.diagnostics.innovationVariance > 0, `Invalid innovation variance at sample ${k}`);
  const covariance = estimate.covariance;
  assert(covariance[0][0] >= 0 && covariance[1][1] >= 0, `Negative covariance diagonal at sample ${k}`);
  assert(Math.abs(covariance[0][1] - covariance[1][0]) < 1e-10, `Covariance lost symmetry at sample ${k}`);
}

const rawPositionRmse = rmse(measurementErrors);
const estimatedPositionRmse = rmse(estimatePositionErrors);
const estimatedVelocityRmse = rmse(estimateVelocityErrors);
const innovationRmse = rmse(innovations);
const covariance = filter.getCovariance();

assert(estimatedPositionRmse < rawPositionRmse * 0.75, `Kalman filter did not improve position RMSE enough: raw=${rawPositionRmse}, estimated=${estimatedPositionRmse}`);
assert(estimatedVelocityRmse < 0.2, `Velocity estimate RMSE is too large: ${estimatedVelocityRmse}`);
assert(covariance.flat().every(Number.isFinite), 'Final covariance contains non-finite values');

console.log('Linear Kalman estimation smoke PASS');
console.log(`Estimation: measurementRMSE=${rawPositionRmse.toFixed(5)}, positionRMSE=${estimatedPositionRmse.toFixed(5)}, velocityRMSE=${estimatedVelocityRmse.toFixed(5)}, innovationRMSE=${innovationRmse.toFixed(5)}`);
