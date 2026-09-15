import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function scenario(enabled) {
  const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
  return {
    ...preset,
    duration: 4.2,
    estimation: {
      ...preset.estimation,
      disturbancePredictionEnabled: true,
      mpcDisturbanceCompensationEnabled: enabled,
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

const off = runSimulation('HYBRID_SAFE', scenario(false));
const on = runSimulation('HYBRID_SAFE', scenario(true));

function row(label, result) {
  const predictionTriggers = result.samples.filter((sample) => sample.triggered && sample.triggerReason === 'prediction-error').length;
  const compensatedSolves = result.solverRecords.filter((record) => record.diagnostics?.disturbanceCompensationEnabled).length;
  return {
    mode: label,
    IAE: result.metrics.iae.toFixed(4),
    effort: result.metrics.controlEffort.toFixed(4),
    solves: result.metrics.solveCount,
    'avoid %': result.metrics.computeReduction.toFixed(1),
    'prediction triggers': predictionTriggers,
    'affine solves': compensatedSolves,
    'conv %': result.metrics.convergenceRate == null ? 'n/a' : result.metrics.convergenceRate.toFixed(1),
    fallback: result.metrics.fallbackCount,
    infeasible: result.metrics.infeasibleCount,
    'QP viol': result.metrics.maxFeasibilityViolation == null ? 'n/a' : result.metrics.maxFeasibilityViolation.toExponential(2),
    'plant safety': result.metrics.maxActualSafetyViolation.toExponential(2),
    xRMSE: result.metrics.estimatePositionRmse?.toFixed(4) ?? 'n/a',
    vRMSE: result.metrics.estimateVelocityRmse?.toFixed(4) ?? 'n/a',
    dRMSE: result.metrics.disturbanceEstimateRmse?.toFixed(4) ?? 'n/a',
  };
}

console.log('Closed-loop affine disturbance MPC compensation probe');
console.table([
  row('MPC d-hat OFF', off),
  row('MPC d-hat ON', on),
]);
console.log(`MPC-comp solve ratio ON/OFF: ${(on.metrics.solveCount / Math.max(1, off.metrics.solveCount)).toFixed(3)}`);
console.log(`MPC-comp IAE ratio ON/OFF: ${(on.metrics.iae / Math.max(1e-12, off.metrics.iae)).toFixed(3)}`);
console.log(`MPC-comp effort ratio ON/OFF: ${(on.metrics.controlEffort / Math.max(1e-12, off.metrics.controlEffort)).toFixed(3)}`);

assert(on.metrics.mpcDisturbanceCompensationEnabled === true, 'MPC disturbance compensation did not activate in the ON scenario.');
assert(on.solverRecords.some((record) => record.diagnostics?.disturbanceCompensationEnabled), 'No solver call reported affine disturbance compensation.');
assert(on.metrics.fallbackCount === 0, `Affine disturbance MPC compensation caused ${on.metrics.fallbackCount} fallback(s).`);
assert(on.metrics.infeasibleCount === 0, `Affine disturbance MPC compensation caused ${on.metrics.infeasibleCount} infeasible solve(s).`);
assert((on.metrics.convergenceRate ?? 0) === 100, `Affine disturbance MPC convergence dropped to ${on.metrics.convergenceRate}%.`);
assert(on.metrics.maxActualSafetyViolation <= 1e-9, `Affine disturbance MPC violated plant safety by ${on.metrics.maxActualSafetyViolation}.`);

console.log('Closed-loop affine disturbance MPC compensation probe PASS');
