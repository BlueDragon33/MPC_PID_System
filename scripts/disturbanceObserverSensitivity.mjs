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
for (const qd of [0.008, 0.016, 0.032, 0.064]) {
  for (const pd of [0.05, 0.2]) candidates.push({ qd, pd });
}

const pulseEnd = 2.25;
const rows = candidates.map(({ qd, pd }) => {
  const result = runSimulation('HYBRID_SAFE', scenario(qd, pd));
  const m = result.metrics;
  const reversalWindow = result.samples.filter((sample) => sample.t >= pulseEnd && sample.t <= 2.9 && sample.disturbanceEstimateEnabled);
  const signMismatchSamples = reversalWindow.filter((sample) => sample.equivalentDisturbance < -0.05 && sample.estimateD > 0);
  const firstNegativeEstimate = reversalWindow.find((sample) => sample.estimateD <= 0);
  const sample240 = result.samples.find((sample) => Math.abs(sample.t - 2.4) < 1e-9);
  return {
    Qd: qd.toExponential(1),
    Pd0: pd.toFixed(2),
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
    'd̂@2.40': sample240?.estimateD?.toFixed(3) ?? '—',
    'deq@2.40': sample240?.equivalentDisturbance?.toFixed(3) ?? '—',
    signMismatch: signMismatchSamples.length,
    signRecovery: firstNegativeEstimate ? (firstNegativeEstimate.t - pulseEnd).toFixed(3) : '>0.65',
    'max Δv': m.maxUncertaintyVelocityMargin.toFixed(4),
  };
});

console.log('Disturbance-observer sign-reversal sensitivity under model mismatch');
console.table(rows);
