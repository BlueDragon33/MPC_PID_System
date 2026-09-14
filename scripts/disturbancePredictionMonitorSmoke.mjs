import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function scenario(predictionEnabled) {
  const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
  return {
    ...preset,
    duration: 4.2,
    estimation: {
      ...preset.estimation,
      disturbancePredictionEnabled: predictionEnabled,
    },
    disturbance: {
      ...preset.disturbance,
      enabled: true,
      start: 1.15,
      duration: 1.1,
      amplitude: 1.0,
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
}

function predictionStats(result) {
  const errors = result.samples.map((sample) => sample.predictionError).filter(Number.isFinite);
  return {
    predictionTriggers: result.samples.filter((sample) => sample.triggered && sample.triggerReason === 'prediction-error').length,
    avgPredictionError: errors.reduce((sum, value) => sum + value, 0) / Math.max(1, errors.length),
    maxPredictionError: errors.length ? Math.max(...errors) : 0,
  };
}

const off = runSimulation('HYBRID_SAFE', scenario(false));
const on = runSimulation('HYBRID_SAFE', scenario(true));
const offStats = predictionStats(off);
const onStats = predictionStats(on);

console.log('Disturbance-aware Event Monitor smoke diagnostics');
console.table([
  {
    mode: 'd-hat OFF',
    IAE: off.metrics.iae.toFixed(4),
    solves: off.metrics.solveCount,
    predictionTriggers: offStats.predictionTriggers,
    avgPredictionError: offStats.avgPredictionError.toFixed(4),
    maxPredictionError: offStats.maxPredictionError.toFixed(4),
    convergence: off.metrics.convergenceRate?.toFixed(1) ?? 'n/a',
    fallback: off.metrics.fallbackCount,
    plantSafety: off.metrics.maxActualSafetyViolation.toExponential(2),
  },
  {
    mode: 'd-hat ON',
    IAE: on.metrics.iae.toFixed(4),
    solves: on.metrics.solveCount,
    predictionTriggers: onStats.predictionTriggers,
    avgPredictionError: onStats.avgPredictionError.toFixed(4),
    maxPredictionError: onStats.maxPredictionError.toFixed(4),
    convergence: on.metrics.convergenceRate?.toFixed(1) ?? 'n/a',
    fallback: on.metrics.fallbackCount,
    plantSafety: on.metrics.maxActualSafetyViolation.toExponential(2),
  },
]);

assert(off.metrics.fallbackCount === 0, 'Observer-only baseline unexpectedly used MPC fallback.');
assert(on.metrics.fallbackCount === 0, `Disturbance-aware monitor caused ${on.metrics.fallbackCount} MPC fallbacks.`);
assert((on.metrics.convergenceRate ?? 100) >= 99.9, `Disturbance-aware monitor convergence dropped to ${on.metrics.convergenceRate}%`);
assert(on.metrics.maxActualSafetyViolation <= 1e-9, `Disturbance-aware monitor violated plant safety: ${on.metrics.maxActualSafetyViolation}`);
assert(on.metrics.solveCount <= off.metrics.solveCount, `Disturbance-aware monitor increased solve count ${off.metrics.solveCount} -> ${on.metrics.solveCount}.`);
assert(onStats.predictionTriggers <= offStats.predictionTriggers, `Prediction-error triggers increased ${offStats.predictionTriggers} -> ${onStats.predictionTriggers}.`);
assert(on.metrics.iae <= off.metrics.iae * 1.01, `Tracking degraded by more than 1%: ${off.metrics.iae} -> ${on.metrics.iae}.`);
assert(onStats.avgPredictionError <= offStats.avgPredictionError * 1.02, `Average prediction error degraded unexpectedly: ${offStats.avgPredictionError} -> ${onStats.avgPredictionError}.`);

console.log('Disturbance-aware Event Monitor smoke PASS');
