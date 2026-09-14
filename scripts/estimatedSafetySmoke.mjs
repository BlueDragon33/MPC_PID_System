import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const preset = applyExperimentPreset(defaultConfig, 'noisy-estimation');
const config = {
  ...preset,
  duration: 2.2,
  disturbance: {
    ...preset.disturbance,
    enabled: true,
    start: 0.7,
    duration: 0.55,
    amplitude: 1.3,
  },
  mpc: {
    ...preset.mpc,
    solver: SOLVER_BACKENDS.CONSTRAINED_QP,
    horizon: 12,
    qpIterations: Math.max(100, preset.mpc.qpIterations),
    qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
    qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
  },
};

assert(config.estimation.enabled, 'Noisy-estimation preset must enable Kalman control.');
assert(config.estimation.constraintTighteningEnabled, 'Noisy-estimation preset must enable covariance tightening.');
assert(config.estimation.constraintSigma === 1.5, `Expected validated 1.5 sigma preset, got ${config.estimation.constraintSigma}`);

const result = runSimulation('HYBRID_SAFE', config);
const metrics = result.metrics;

assert(metrics.estimationEnabled, 'Estimated-state closed loop did not report estimation metrics.');
assert(metrics.uncertaintyTighteningEnabled, 'Covariance tightening did not activate.');
assert(metrics.measurementRmse > 0, 'Measurement RMSE must be positive under noisy sensor simulation.');
assert(metrics.estimatePositionRmse < metrics.measurementRmse, `Kalman position estimate must improve measurement RMSE: measurement=${metrics.measurementRmse}, estimate=${metrics.estimatePositionRmse}`);
assert(Number.isFinite(metrics.estimateVelocityRmse), 'Velocity estimate RMSE must remain finite.');
assert(metrics.maxActualSafetyViolation <= 1e-9, `Estimated-state plant left the physical safety envelope: ${metrics.maxActualSafetyViolation}`);
assert(metrics.safetyViolationCount === 0, `Estimated-state run contains ${metrics.safetyViolationCount} unsafe samples.`);
assert(metrics.safetyViolationRate === 0, `Estimated-state unsafe-sample rate must be zero, got ${metrics.safetyViolationRate}%`);
assert(metrics.fallbackCount === 0, `Validated estimated-state scenario must not require MPC fallback, got ${metrics.fallbackCount}`);
assert(metrics.infeasibleCount === 0, `Validated estimated-state scenario must not report infeasible QPs, got ${metrics.infeasibleCount}`);
assert(metrics.numericalFailureCount === 0, 'Validated estimated-state scenario must not report numerical failures.');
assert(metrics.timeoutCount === 0, 'Validated estimated-state scenario must not report solver timeouts.');
assert(metrics.convergenceRate === 100, `Validated estimated-state scenario requires 100% QP convergence, got ${metrics.convergenceRate}`);
assert(metrics.uncertaintyInvalidEnvelopeCount === 0, `Covariance tightening produced ${metrics.uncertaintyInvalidEnvelopeCount} invalid robust envelopes.`);
assert(metrics.maxUncertaintyPositionMargin > 0, 'Position uncertainty margin should be active.');
assert(metrics.maxUncertaintyVelocityMargin > 0, 'Velocity uncertainty margin should be active.');
assert(result.samples.every((sample) => Number.isFinite(sample.u) && Number.isFinite(sample.x) && Number.isFinite(sample.v)), 'Estimated-state safety run contains non-finite plant/control samples.');

console.log('Estimated-state covariance safety smoke PASS');
console.log(`EstimatedSafety: kσ=${config.estimation.constraintSigma.toFixed(1)}, IAE=${metrics.iae.toFixed(4)}, measurementRMSE=${metrics.measurementRmse.toFixed(4)}, xHatRMSE=${metrics.estimatePositionRmse.toFixed(4)}, vHatRMSE=${metrics.estimateVelocityRmse.toFixed(4)}, solves=${metrics.solveCount}, convergence=${metrics.convergenceRate.toFixed(1)}%, fallback=${metrics.fallbackCount}, plantViolation=${metrics.maxActualSafetyViolation.toExponential(2)}`);
