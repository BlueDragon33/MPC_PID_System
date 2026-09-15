import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function scenario(kind, compensationEnabled) {
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
      ? { ...preset.disturbance, enabled: true, start: 1.15, duration: 1.1, amplitude: kind === 'additive-only' ? 1.5 : 1.0 }
      : { ...preset.disturbance, enabled: false, amplitude: 0 },
    mpc: {
      ...preset.mpc,
      solver: SOLVER_BACKENDS.CONSTRAINED_QP,
      horizon: 16,
      qpIterations: Math.max(100, preset.mpc.qpIterations),
      qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
      qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
    },
  };
}

const rows = [];
for (const kind of ['additive-only', 'parameter-only', 'combined']) {
  for (const enabled of [false, true]) {
    const result = runSimulation('HYBRID_SAFE', scenario(kind, enabled));
    rows.push({ kind, enabled, result });
  }
}

function summarize({ kind, enabled, result }) {
  return {
    scenario: kind,
    compensation: enabled ? 'ON' : 'OFF',
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

console.log('Affine disturbance compensation matrix');
console.table(rows.map(summarize));

for (const kind of ['additive-only', 'parameter-only', 'combined']) {
  const off = rows.find((row) => row.kind === kind && !row.enabled).result;
  const on = rows.find((row) => row.kind === kind && row.enabled).result;
  console.log(`${kind}: IAE ratio=${(on.metrics.iae / Math.max(1e-12, off.metrics.iae)).toFixed(4)}, effort ratio=${(on.metrics.controlEffort / Math.max(1e-12, off.metrics.controlEffort)).toFixed(4)}, solve ratio=${(on.metrics.solveCount / Math.max(1, off.metrics.solveCount)).toFixed(4)}`);
  assert(on.metrics.fallbackCount === 0, `${kind}: compensation caused fallback.`);
  assert(on.metrics.infeasibleCount === 0, `${kind}: compensation caused infeasible solve.`);
  assert((on.metrics.convergenceRate ?? 0) === 100, `${kind}: compensation convergence dropped below 100%.`);
  assert(on.metrics.maxActualSafetyViolation <= 1e-9, `${kind}: compensation violated plant safety.`);
}

console.log('Affine disturbance compensation matrix PASS');
