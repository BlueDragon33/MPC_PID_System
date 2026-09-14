import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';

function scenario(qd, pd) {
  const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
  return {
    ...preset,
    duration: 4.2,
    estimation: {
      ...preset.estimation,
      disturbanceProcessVariance: qd,
      initialDisturbanceVariance: pd,
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
      horizon: 12,
      qpIterations: Math.max(100, preset.mpc.qpIterations),
      qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
      qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
    },
  };
}

const candidates = [];
for (const qd of [0.001, 0.002, 0.004, 0.008]) {
  for (const pd of [0.2, 0.8]) candidates.push({ qd, pd });
}

const rows = candidates.map(({ qd, pd }) => {
  const result = runSimulation('HYBRID_SAFE', scenario(qd, pd));
  const m = result.metrics;
  return {
    Qd: qd.toExponential(1),
    Pd0: pd.toFixed(1),
    IAE: m.iae.toFixed(4),
    solves: m.solveCount,
    'conv %': m.convergenceRate == null ? 'n/a' : m.convergenceRate.toFixed(1),
    fallback: m.fallbackCount,
    infeasible: m.infeasibleCount,
    'plant safety': m.maxActualSafetyViolation.toExponential(2),
    xRMSE: m.estimatePositionRmse?.toFixed(4) ?? '—',
    vRMSE: m.estimateVelocityRmse?.toFixed(4) ?? '—',
    dRMSE: m.disturbanceEstimateRmse?.toFixed(4) ?? '—',
    activeDRMSE: m.activeDisturbanceEstimateRmse?.toFixed(4) ?? '—',
    'max Δv': m.maxUncertaintyVelocityMargin.toFixed(4),
  };
});

console.log('Disturbance-observer covariance sensitivity under model mismatch');
console.table(rows);
