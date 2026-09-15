import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function scenario(kind, compensationEnabled, amplitude = 1.0) {
  const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
  const additive = kind === 'additive-only' || kind === 'combined';
  const mismatch = kind === 'parameter-only' || kind === 'combined';
  return {
    ...preset,
    duration: 4.2,
    truthPlant: mismatch
      ? { ...preset.truthPlant, enabled: true }
      : { ...preset.truthPlant, enabled: false, stiffnessScale: 1, dampingScale: 1, gainScale: 1 },
    estimation: {
      ...preset.estimation,
      disturbancePredictionEnabled: true,
      mpcDisturbanceCompensationEnabled: compensationEnabled,
    },
    disturbance: additive
      ? { ...preset.disturbance, enabled: true, start: 1.15, duration: 1.1, amplitude }
      : { ...preset.disturbance, enabled: false, amplitude: 0 },
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

function isStrictlyFeasible(result) {
  return result.metrics.fallbackCount === 0
    && result.metrics.infeasibleCount === 0
    && (result.metrics.convergenceRate ?? 0) === 100
    && result.metrics.maxActualSafetyViolation <= 1e-9;
}

function summarize(label, result, amplitude = null) {
  return {
    scenario: label,
    amplitude: amplitude == null ? '—' : amplitude.toFixed(2),
    IAE: result.metrics.iae.toFixed(4),
    effort: result.metrics.controlEffort.toFixed(4),
    solves: result.metrics.solveCount,
    'avoid %': result.metrics.computeReduction.toFixed(1),
    'conv %': result.metrics.convergenceRate == null ? 'n/a' : result.metrics.convergenceRate.toFixed(1),
    fallback: result.metrics.fallbackCount,
    infeasible: result.metrics.infeasibleCount,
    'plant safety': result.metrics.maxActualSafetyViolation.toExponential(2),
    xRMSE: result.metrics.estimatePositionRmse?.toFixed(4) ?? 'n/a',
    vRMSE: result.metrics.estimateVelocityRmse?.toFixed(4) ?? 'n/a',
    dRMSE: result.metrics.disturbanceEstimateRmse?.toFixed(4) ?? 'n/a',
  };
}

// Predeclared disturbance grid: determine the largest additive pulse for which
// the OFF baseline remains fully feasible/safe. Compensation is evaluated at
// exactly that same operating point, not at a separately tuned amplitude.
const additiveGrid = [0.2, 0.4, 0.6, 0.8, 1.0];
const additiveFrontierRows = additiveGrid.map((amplitude) => ({
  amplitude,
  result: runSimulation('HYBRID_SAFE', scenario('additive-only', false, amplitude)),
}));
const feasibleAdditiveRows = additiveFrontierRows.filter(({ result }) => isStrictlyFeasible(result));
const additiveFrontier = feasibleAdditiveRows[feasibleAdditiveRows.length - 1] ?? null;

console.log('Additive-disturbance feasible-frontier sweep (compensation OFF)');
console.table(additiveFrontierRows.map(({ amplitude, result }) => summarize('additive OFF', result, amplitude)));
assert(additiveFrontier, 'No strictly feasible additive-disturbance point found on the predeclared 0.2–1.0 grid.');

const additiveAmplitude = additiveFrontier.amplitude;
const additiveOff = additiveFrontier.result;
const additiveOn = runSimulation('HYBRID_SAFE', scenario('additive-only', true, additiveAmplitude));
const parameterOff = runSimulation('HYBRID_SAFE', scenario('parameter-only', false, 0));
const parameterOn = runSimulation('HYBRID_SAFE', scenario('parameter-only', true, 0));
const combinedOff = runSimulation('HYBRID_SAFE', scenario('combined', false, 1.0));
const combinedOn = runSimulation('HYBRID_SAFE', scenario('combined', true, 1.0));

const comparisons = [
  { kind: 'additive-only', amplitude: additiveAmplitude, off: additiveOff, on: additiveOn },
  { kind: 'parameter-only', amplitude: null, off: parameterOff, on: parameterOn },
  { kind: 'combined', amplitude: 1.0, off: combinedOff, on: combinedOn },
];

console.log(`Selected additive feasible frontier: amplitude=${additiveAmplitude.toFixed(2)}`);
console.log('Affine disturbance compensation matrix at valid operating points');
console.table(comparisons.flatMap(({ kind, amplitude, off, on }) => [
  { ...summarize(kind, off, amplitude), compensation: 'OFF' },
  { ...summarize(kind, on, amplitude), compensation: 'ON' },
]));

for (const { kind, off, on } of comparisons) {
  console.log(`${kind}: IAE ratio=${(on.metrics.iae / Math.max(1e-12, off.metrics.iae)).toFixed(4)}, effort ratio=${(on.metrics.controlEffort / Math.max(1e-12, off.metrics.controlEffort)).toFixed(4)}, solve ratio=${(on.metrics.solveCount / Math.max(1, off.metrics.solveCount)).toFixed(4)}`);

  assert(isStrictlyFeasible(off), `${kind}: OFF baseline is not strictly feasible/safe at the selected operating point.`);
  assert(isStrictlyFeasible(on), `${kind}: affine compensation is not strictly feasible/safe at the same operating point.`);
}

console.log('Affine disturbance compensation matrix PASS');
