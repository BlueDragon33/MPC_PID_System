import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function shortScenario(id) {
  const preset = applyExperimentPreset(defaultConfig, id);
  return {
    ...preset,
    duration: 4.2,
    disturbance: {
      ...preset.disturbance,
      enabled: true,
      start: 1.15,
      duration: 1.1,
      amplitude: 1.0,
    },
    mpc: {
      ...preset.mpc,
      horizon: 12,
      qpIterations: Math.max(100, preset.mpc.qpIterations),
      qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
      qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
    },
  };
}

const stateOnly = runSimulation('HYBRID_SAFE', shortScenario('model-mismatch'));
const augmented = runSimulation('HYBRID_SAFE', shortScenario('mismatch-observer'));

assert(stateOnly.metrics.truthPlantMismatchEnabled, 'State-only case did not activate truth/model mismatch.');
assert(augmented.metrics.truthPlantMismatchEnabled, 'Augmented case did not activate truth/model mismatch.');
assert(!stateOnly.metrics.disturbanceEstimateEnabled, 'State-only case unexpectedly enabled disturbance estimation.');
assert(augmented.metrics.disturbanceEstimateEnabled, 'Augmented disturbance estimator was not routed into simulator diagnostics.');
assert(Number.isFinite(augmented.metrics.disturbanceEstimateRmse), 'Disturbance RMSE is not finite.');
assert(augmented.metrics.disturbanceEstimateRmse < 0.55, `Equivalent-disturbance RMSE too high: ${augmented.metrics.disturbanceEstimateRmse}`);
assert(Number.isFinite(augmented.metrics.estimatePositionRmse), 'Augmented x-hat RMSE is not finite.');
assert(Number.isFinite(augmented.metrics.estimateVelocityRmse), 'Augmented v-hat RMSE is not finite.');
assert(augmented.metrics.estimatePositionRmse <= stateOnly.metrics.estimatePositionRmse * 1.35, 'Augmented observer degraded position estimation excessively.');
assert(augmented.metrics.estimateVelocityRmse <= stateOnly.metrics.estimateVelocityRmse * 1.35, 'Augmented observer degraded velocity estimation excessively.');
assert(augmented.metrics.maxActualSafetyViolation <= 1e-9, `Augmented observer case violated plant safety: ${augmented.metrics.maxActualSafetyViolation}`);
assert(augmented.metrics.fallbackCount === 0, `Augmented observer caused ${augmented.metrics.fallbackCount} MPC fallbacks.`);
assert((augmented.metrics.convergenceRate ?? 100) >= 99.9, `Augmented observer convergence dropped to ${augmented.metrics.convergenceRate}%`);

console.log('Model-mismatch disturbance observer smoke PASS');
console.table([
  {
    estimator: '2-state',
    IAE: stateOnly.metrics.iae.toFixed(4),
    solves: stateOnly.metrics.solveCount,
    xRMSE: stateOnly.metrics.estimatePositionRmse?.toFixed(4),
    vRMSE: stateOnly.metrics.estimateVelocityRmse?.toFixed(4),
    dRMSE: '—',
    plantSafety: stateOnly.metrics.maxActualSafetyViolation.toExponential(2),
    fallback: stateOnly.metrics.fallbackCount,
  },
  {
    estimator: 'x-v-d',
    IAE: augmented.metrics.iae.toFixed(4),
    solves: augmented.metrics.solveCount,
    xRMSE: augmented.metrics.estimatePositionRmse?.toFixed(4),
    vRMSE: augmented.metrics.estimateVelocityRmse?.toFixed(4),
    dRMSE: augmented.metrics.disturbanceEstimateRmse?.toFixed(4),
    activeDRMSE: augmented.metrics.activeDisturbanceEstimateRmse?.toFixed(4),
    plantSafety: augmented.metrics.maxActualSafetyViolation.toExponential(2),
    fallback: augmented.metrics.fallbackCount,
  },
]);
