import { runUgvSimulation, UGV_CONTROLLER_MODES } from '../src/core/orchestration/ugvSimulator.js';

const horizons = [6, 8, 10, 12, 14];
const rows = [];

for (const horizon of horizons) {
  const result = runUgvSimulation(UGV_CONTROLLER_MODES.LTV_MPC, {
    duration: 6,
    ugvEstimation: { enabled: false },
    ugvLtvMpc: { horizon },
  });
  const m = result.metrics;
  const residuals = result.solverRecords
    .map((record) => record.diagnostics?.projectedGradientResidual)
    .filter(Number.isFinite);
  const avgResidual = residuals.length
    ? residuals.reduce((sum, value) => sum + value, 0) / residuals.length
    : 0;
  const maxResidual = residuals.length ? Math.max(...residuals) : 0;
  const repairedSolves = result.solverRecords.filter(
    (record) => (record.diagnostics?.anchorRepairs ?? 0) > 0,
  ).length;
  rows.push({
    horizon,
    horizonSeconds: (horizon * result.config.ugvLtvMpc.predictionDt).toFixed(2),
    lateralRMSE: m.lateralRmse.toFixed(4),
    headingRMSE: m.headingRmse.toFixed(4),
    speedRMSE: m.speedRmse.toFixed(4),
    effort: m.controlEffort.toFixed(4),
    safety: m.maxSafetyViolation.toExponential(2),
    solves: m.solveCount,
    solvedPct: m.convergenceRate?.toFixed(1) ?? '—',
    acceptedPct: m.acceptedRate?.toFixed(1) ?? '—',
    fallback: m.fallbackCount,
    avgResidual: avgResidual.toExponential(2),
    maxResidual: maxResidual.toExponential(2),
    anchorRepair: repairedSolves,
    avgSolveMs: m.averageSolveMs.toFixed(2),
    maxSolveMs: m.maxSolveMs.toFixed(2),
  });
}

console.log('UGV LTV horizon sensitivity');
console.table(rows);
