import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

const cases = [
  ...[SOLVER_BACKENDS.CONSTRAINED_QP, SOLVER_BACKENDS.BOX_QP, SOLVER_BACKENDS.PROJECTED_GRADIENT]
    .map((backend) => ({ preset: 'baseline', mode: 'MPC', backend })),
  { preset: 'baseline', mode: 'HYBRID', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'rate-limited', mode: 'MPC', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'MPC', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'HYBRID', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'HYBRID', backend: SOLVER_BACKENDS.BOX_QP },
  { preset: 'disturbance-stress', mode: 'HYBRID', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
];

const rows = [];
for (const item of cases) {
  const presetConfig = applyExperimentPreset(defaultConfig, item.preset);
  const config = {
    ...presetConfig,
    duration: Math.min(4, presetConfig.duration),
    mpc: {
      ...presetConfig.mpc,
      solver: item.backend,
      horizon: Math.min(24, presetConfig.mpc.horizon),
    },
  };
  const result = runSimulation(item.mode, config);
  rows.push({
    ...item,
    iae: result.metrics.iae,
    solveCount: result.metrics.solveCount,
    computeReduction: result.metrics.computeReduction,
    avgSolveMs: result.metrics.avgSolveMs,
    maxSolveMs: result.metrics.maxSolveMs,
    convergenceRate: result.metrics.convergenceRate,
    fallbackCount: result.metrics.fallbackCount,
    maxQPViolation: result.metrics.maxFeasibilityViolation,
    actualSafetyViolation: result.metrics.maxActualSafetyViolation,
    safetyViolationRate: result.metrics.safetyViolationRate,
  });
}

console.log('MPC_PID_System solver + safety authority benchmark');
console.table(rows.map((row) => ({
  preset: row.preset,
  mode: row.mode,
  backend: row.backend,
  IAE: row.iae.toFixed(4),
  solves: row.solveCount,
  'avoid %': row.computeReduction.toFixed(1),
  'avg ms': row.avgSolveMs.toFixed(3),
  'conv %': row.convergenceRate == null ? 'n/a' : row.convergenceRate.toFixed(1),
  fallback: row.fallbackCount,
  'QP viol': row.maxQPViolation == null ? 'n/a' : row.maxQPViolation.toExponential(2),
  'plant safety': row.actualSafetyViolation.toExponential(2),
  'unsafe %': row.safetyViolationRate.toFixed(1),
})));

const constrainedSafetyMpc = rows.find((row) => row.preset === 'safety-envelope' && row.mode === 'MPC' && row.backend === SOLVER_BACKENDS.CONSTRAINED_QP);
const constrainedSafetyHybrid = rows.find((row) => row.preset === 'safety-envelope' && row.mode === 'HYBRID' && row.backend === SOLVER_BACKENDS.CONSTRAINED_QP);
if (constrainedSafetyMpc && constrainedSafetyHybrid) {
  console.log('\nSafety authority comparison');
  console.log(`Periodic constrained MPC plant violation: ${constrainedSafetyMpc.actualSafetyViolation.toExponential(3)}`);
  console.log(`Hybrid guidance-only plant violation:      ${constrainedSafetyHybrid.actualSafetyViolation.toExponential(3)}`);
  console.log('A non-zero hybrid gap is evidence for adding a Safety Governor / admissibility filter rather than assuming predicted feasibility automatically transfers through PID.');
}
