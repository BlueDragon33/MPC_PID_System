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

function diagnosticRow(label, result) {
  const m = result.metrics;
  return {
    estimator: label,
    IAE: m.iae.toFixed(4),
    solves: m.solveCount,
    'conv %': m.convergenceRate == null ? 'n/a' : m.convergenceRate.toFixed(1),
    fallback: m.fallbackCount,
    infeasible: m.infeasibleCount,
    timeout: m.timeoutCount,
    numerical: m.numericalFailureCount,
    'QP viol': m.maxFeasibilityViolation == null ? 'n/a' : m.maxFeasibilityViolation.toExponential(2),
    'plant safety': m.maxActualSafetyViolation.toExponential(2),
    'unsafe %': m.safetyViolationRate.toFixed(1),
    xRMSE: m.estimatePositionRmse?.toFixed(4) ?? '—',
    vRMSE: m.estimateVelocityRmse?.toFixed(4) ?? '—',
    dRMSE: m.disturbanceEstimateRmse?.toFixed(4) ?? '—',
    activeDRMSE: m.activeDisturbanceEstimateRmse?.toFixed(4) ?? '—',
    'max Δx': m.maxUncertaintyPositionMargin.toFixed(4),
    'max Δv': m.maxUncertaintyVelocityMargin.toFixed(4),
    'invalid envelope': m.uncertaintyInvalidEnvelopeCount,
  };
}

const stateOnly = runSimulation('HYBRID_SAFE', shortScenario('model-mismatch'));
const augmented = runSimulation('HYBRID_SAFE', shortScenario('mismatch-observer'));

console.log('Model-mismatch disturbance observer diagnostics');
console.table([
  diagnosticRow('2-state', stateOnly),
  diagnosticRow('x-v-d', augmented),
]);

const augmentedFailures = augmented.solverRecords
  .filter((record) => record.fallbackUsed || record.status !== 'solved')
  .map((record, index) => ({
    index,
    status: record.status,
    fallback: record.fallbackUsed,
    reason: record.fallbackReason ?? '',
    converged: record.diagnostics?.converged ?? null,
    iterations: record.diagnostics?.iterations ?? null,
    feasibility: record.diagnostics?.feasibilityViolation ?? null,
    residual: record.diagnostics?.projectedGradientResidual ?? record.diagnostics?.kktResidual ?? null,
  }));
if (augmentedFailures.length) {
  console.log('Augmented observer non-solved/fallback MPC calls');
  console.table(augmentedFailures);
}

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
