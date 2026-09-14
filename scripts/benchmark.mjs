import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

const cases = [
  ...[SOLVER_BACKENDS.CONSTRAINED_QP, SOLVER_BACKENDS.BOX_QP, SOLVER_BACKENDS.PROJECTED_GRADIENT]
    .map((backend) => ({ preset: 'baseline', mode: 'MPC', backend })),
  { preset: 'baseline', mode: 'HYBRID', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'baseline', mode: 'HYBRID_SAFE', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'rate-limited', mode: 'MPC', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'MPC', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'HYBRID', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'HYBRID_SAFE', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'safety-envelope', mode: 'HYBRID', backend: SOLVER_BACKENDS.BOX_QP },
  { preset: 'disturbance-stress', mode: 'HYBRID', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
  { preset: 'disturbance-stress', mode: 'HYBRID_SAFE', backend: SOLVER_BACKENDS.CONSTRAINED_QP },
];

const rows = [];
for (const item of cases) {
  const presetConfig = applyExperimentPreset(defaultConfig, item.preset);
  const generalEnvelope = item.backend === SOLVER_BACKENDS.CONSTRAINED_QP
    && (presetConfig.mpc.stateConstraintsEnabled || presetConfig.mpc.outputConstraintsEnabled);
  const stress = item.preset === 'disturbance-stress';

  const config = {
    ...presetConfig,
    duration: generalEnvelope ? 1.6 : Math.min(2.0, presetConfig.duration),
    disturbance: stress
      ? {
          ...presetConfig.disturbance,
          enabled: true,
          start: 0.55,
          duration: 0.65,
        }
      : { ...presetConfig.disturbance },
    safety: {
      ...defaultConfig.safety,
      ...(presetConfig.safety || {}),
    },
    mpc: {
      ...presetConfig.mpc,
      solver: item.backend,
      horizon: generalEnvelope ? Math.min(12, presetConfig.mpc.horizon) : Math.min(16, presetConfig.mpc.horizon),
      qpIterations: generalEnvelope ? Math.max(100, presetConfig.mpc.qpIterations) : Math.min(40, presetConfig.mpc.qpIterations),
      qpProjectionCycles: generalEnvelope ? Math.max(16, presetConfig.mpc.qpProjectionCycles) : Math.min(10, presetConfig.mpc.qpProjectionCycles),
      qpTolerance: generalEnvelope ? Math.min(5e-5, presetConfig.mpc.qpTolerance) : presetConfig.mpc.qpTolerance,
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
    governorInterventionRate: result.metrics.governorInterventionRate,
    governorConditioningRate: result.metrics.governorConditioningRate,
    governorMaxCorrection: result.metrics.governorMaxCorrection,
    governorMaxConditioningCorrection: result.metrics.governorMaxConditioningCorrection,
    governorEmergencyCount: result.metrics.governorEmergencyCount,
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
  'safety %': row.governorInterventionRate.toFixed(1),
  'condition %': row.governorConditioningRate.toFixed(1),
  'safety max Δu': row.governorMaxCorrection.toFixed(3),
  'condition max Δu': row.governorMaxConditioningCorrection.toFixed(3),
  emergency: row.governorEmergencyCount,
})));

const constrainedSafetyMpc = rows.find((row) => row.preset === 'safety-envelope' && row.mode === 'MPC' && row.backend === SOLVER_BACKENDS.CONSTRAINED_QP);
const constrainedSafetyHybrid = rows.find((row) => row.preset === 'safety-envelope' && row.mode === 'HYBRID' && row.backend === SOLVER_BACKENDS.CONSTRAINED_QP);
const constrainedSafetyGoverned = rows.find((row) => row.preset === 'safety-envelope' && row.mode === 'HYBRID_SAFE' && row.backend === SOLVER_BACKENDS.CONSTRAINED_QP);
if (constrainedSafetyMpc && constrainedSafetyHybrid && constrainedSafetyGoverned) {
  console.log('\nSafety authority comparison');
  console.log(`Periodic constrained MPC plant violation: ${constrainedSafetyMpc.actualSafetyViolation.toExponential(3)}`);
  console.log(`Hybrid guidance-only plant violation:      ${constrainedSafetyHybrid.actualSafetyViolation.toExponential(3)}`);
  console.log(`Hybrid + Safety Governor violation:        ${constrainedSafetyGoverned.actualSafetyViolation.toExponential(3)}`);
  console.log(`Safety-envelope intervention rate:         ${constrainedSafetyGoverned.governorInterventionRate.toFixed(1)}%`);
  console.log(`Actuator/plan conditioning rate:           ${constrainedSafetyGoverned.governorConditioningRate.toFixed(1)}%`);
}
