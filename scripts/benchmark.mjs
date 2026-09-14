import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

const backends = [
  SOLVER_BACKENDS.CONSTRAINED_QP,
  SOLVER_BACKENDS.BOX_QP,
  SOLVER_BACKENDS.PROJECTED_GRADIENT,
];

const scenarios = ['baseline', 'rate-limited', 'safety-envelope'];
const rows = [];

for (const presetId of scenarios) {
  const presetConfig = applyExperimentPreset(defaultConfig, presetId);
  for (const backend of backends) {
    const config = {
      ...presetConfig,
      duration: Math.min(4, presetConfig.duration),
      mpc: {
        ...presetConfig.mpc,
        solver: backend,
        horizon: Math.min(24, presetConfig.mpc.horizon),
      },
    };
    const result = runSimulation('MPC', config);
    rows.push({
      preset: presetId,
      backend,
      iae: result.metrics.iae,
      avgSolveMs: result.metrics.avgSolveMs,
      maxSolveMs: result.metrics.maxSolveMs,
      convergenceRate: result.metrics.convergenceRate,
      fallbackCount: result.metrics.fallbackCount,
      maxQPViolation: result.metrics.maxFeasibilityViolation,
      actualSafetyViolation: result.metrics.maxActualSafetyViolation,
      safetyViolationRate: result.metrics.safetyViolationRate,
    });
  }
}

console.log('MPC_PID_System solver benchmark');
console.table(rows.map((row) => ({
  preset: row.preset,
  backend: row.backend,
  IAE: row.iae.toFixed(4),
  'avg ms': row.avgSolveMs.toFixed(3),
  'max ms': row.maxSolveMs.toFixed(3),
  'conv %': row.convergenceRate == null ? 'n/a' : row.convergenceRate.toFixed(1),
  fallback: row.fallbackCount,
  'QP viol': row.maxQPViolation == null ? 'n/a' : row.maxQPViolation.toExponential(2),
  'plant safety': row.actualSafetyViolation.toExponential(2),
  'unsafe %': row.safetyViolationRate.toFixed(1),
})));

console.log('\nInterpretation: box-qp and projected-gradient are retained as baselines. They do not enforce the general state/output polyhedron. Compare both QP feasibility and real-plant safety metrics; neither timing nor tracking error alone is sufficient.');
