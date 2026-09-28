import { runUgvSimulation, UGV_CONTROLLER_MODES } from '../src/core/orchestration/ugvSimulator.js';

function cfg(estimationEnabled) {
  return {
    duration: 8,
    ugvEstimation: { enabled: estimationEnabled, seed: 20260928 },
  };
}

function row(label, result) {
  const m = result.metrics;
  return {
    case: label,
    lateralRMSE: m.lateralRmse.toFixed(4),
    maxLateral: m.maxAbsLateralError.toFixed(4),
    headingRMSE: m.headingRmse.toFixed(4),
    speedRMSE: m.speedRmse.toFixed(4),
    effort: m.controlEffort.toFixed(4),
    safety: m.maxSafetyViolation.toExponential(2),
    unsafe: m.unsafeSamples,
    solves: m.solveCount,
    conv: m.convergenceRate == null ? '—' : m.convergenceRate.toFixed(1),
    accepted: m.acceptedRate == null ? '—' : m.acceptedRate.toFixed(1),
    fallback: m.fallbackCount,
    avgSolveMs: m.averageSolveMs.toFixed(2),
    maxSolveMs: m.maxSolveMs.toFixed(2),
    xRMSE: m.estimateXRmse?.toFixed(4) ?? '—',
    yRMSE: m.estimateYRmse?.toFixed(4) ?? '—',
    yawRMSE: m.estimateYawRmse?.toFixed(4) ?? '—',
    vRMSE: m.estimateSpeedRmse?.toFixed(4) ?? '—',
    progress: m.finalProgressX.toFixed(2),
  };
}

const classicalTruth = runUgvSimulation(UGV_CONTROLLER_MODES.CLASSICAL, cfg(false));
const classicalEkf = runUgvSimulation(UGV_CONTROLLER_MODES.CLASSICAL, cfg(true));
const ltvTruth = runUgvSimulation(UGV_CONTROLLER_MODES.LTV_MPC, cfg(false));
const ltvEkf = runUgvSimulation(UGV_CONTROLLER_MODES.LTV_MPC, cfg(true));

console.log('UGV Gate 5A closed-loop probe');
console.table([
  row('Classical truth', classicalTruth),
  row('Classical EKF', classicalEkf),
  row('LTV-MPC truth', ltvTruth),
  row('LTV-MPC EKF', ltvEkf),
]);

console.log('UGV comparison ratios', {
  ltvVsClassicalLateralTruth: ltvTruth.metrics.lateralRmse / classicalTruth.metrics.lateralRmse,
  ltvVsClassicalLateralEkf: ltvEkf.metrics.lateralRmse / classicalEkf.metrics.lateralRmse,
  ltvVsClassicalEffortTruth: ltvTruth.metrics.controlEffort / classicalTruth.metrics.controlEffort,
  ltvVsClassicalEffortEkf: ltvEkf.metrics.controlEffort / classicalEkf.metrics.controlEffort,
});
