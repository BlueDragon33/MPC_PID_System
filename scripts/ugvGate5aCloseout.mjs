import assert from 'node:assert/strict';

import {
  runUgvSimulation,
  UGV_CONTROLLER_MODES,
} from '../src/core/orchestration/ugvSimulator.js';
import { mergeUgvConfig } from '../src/core/orchestration/ugvSimulationConfig.js';

const closeoutConfig = {
  duration: 8,
  ugvEstimation: {
    enabled: true,
    seed: 20260928,
  },
};

function run(mode, estimationEnabled) {
  return runUgvSimulation(mode, {
    ...closeoutConfig,
    ugvEstimation: {
      ...closeoutConfig.ugvEstimation,
      enabled: estimationEnabled,
    },
  });
}

function assertFiniteTrace(result, label) {
  const fields = [
    't', 'x', 'y', 'yaw', 'v',
    'referenceX', 'referenceY', 'referenceYaw', 'referenceSpeed',
    'lateralError', 'headingError', 'speedError',
    'acceleration', 'steering', 'safetyViolation',
  ];
  for (const [index, sample] of result.samples.entries()) {
    for (const field of fields) {
      assert(Number.isFinite(sample[field]), `${label}: non-finite ${field} at sample ${index}`);
    }
  }
}

function deterministicTrace(result) {
  return result.samples.map((sample) => [
    sample.x,
    sample.y,
    sample.yaw,
    sample.v,
    sample.estimateX,
    sample.estimateY,
    sample.estimateYaw,
    sample.estimateV,
    sample.acceleration,
    sample.steering,
    sample.triggered,
    sample.solverStatus,
    sample.fallbackUsed,
  ]);
}

function strictSafety(result, label) {
  assert.equal(result.metrics.unsafeSamples, 0, `${label}: unsafeSamples=${result.metrics.unsafeSamples}`);
  assert(
    result.metrics.maxSafetyViolation <= 1e-12,
    `${label}: maxSafetyViolation=${result.metrics.maxSafetyViolation}`,
  );
}

function strictPredictiveNumerics(result, label) {
  const m = result.metrics;
  assert.equal(m.fallbackCount, 0, `${label}: fallbackCount=${m.fallbackCount}`);
  assert.equal(m.acceptedRate, 100, `${label}: acceptedRate=${m.acceptedRate}`);
  assert(
    m.maxFeasibilityViolation <= 1e-6,
    `${label}: maxFeasibilityViolation=${m.maxFeasibilityViolation}`,
  );
  assert(
    m.averageOptimalityResidual <= 5e-3,
    `${label}: averageOptimalityResidual=${m.averageOptimalityResidual}`,
  );
  assert(
    m.maxOptimalityResidual <= 5e-2,
    `${label}: maxOptimalityResidual=${m.maxOptimalityResidual}`,
  );
  assert(
    m.averageSolveMs < 100,
    `${label}: average CI solve time ${m.averageSolveMs}ms exceeds research budget`,
  );
  assert(
    m.maxSolveMs < 250,
    `${label}: max CI solve time ${m.maxSolveMs}ms exceeds research budget`,
  );
}

const cfg = mergeUgvConfig(closeoutConfig);
assert.equal(cfg.ugvLtvMpc.horizon, 6, 'Gate 5A closeout requires Pareto-selected horizon 6.');
assert.equal(cfg.ugvLtvMpc.predictionDt, 0.10, 'UGV prediction stage must be 0.10 s.');
assert.equal(cfg.ugvLtvMpc.solveInterval, 0.10, 'UGV MPC solve cadence must match prediction stage.');
assert.equal(cfg.dt, 0.05, 'UGV plant integration cadence must remain 0.05 s.');

const classicalTruth = run(UGV_CONTROLLER_MODES.CLASSICAL, false);
const classicalEkf = run(UGV_CONTROLLER_MODES.CLASSICAL, true);
const ltvTruth = run(UGV_CONTROLLER_MODES.LTV_MPC, false);
const ltvEkf = run(UGV_CONTROLLER_MODES.LTV_MPC, true);
const ltvEkfRepeat = run(UGV_CONTROLLER_MODES.LTV_MPC, true);

for (const [label, result] of [
  ['classical-truth', classicalTruth],
  ['classical-ekf', classicalEkf],
  ['ltv-truth', ltvTruth],
  ['ltv-ekf', ltvEkf],
]) {
  assertFiniteTrace(result, label);
  strictSafety(result, label);
  assert(result.metrics.finalProgressX > 30, `${label}: insufficient forward progress`);
}

strictPredictiveNumerics(ltvTruth, 'ltv-truth');
strictPredictiveNumerics(ltvEkf, 'ltv-ekf');

assert(
  (ltvTruth.metrics.convergenceRate ?? 0) >= 90,
  `LTV truth solved-rate dropped to ${ltvTruth.metrics.convergenceRate}%`,
);
assert(
  (ltvEkf.metrics.convergenceRate ?? 0) >= 60,
  `LTV EKF solved-rate dropped to ${ltvEkf.metrics.convergenceRate}%`,
);

assert(
  ltvTruth.metrics.lateralRmse <= classicalTruth.metrics.lateralRmse * 1.05,
  'LTV truth lateral tracking degraded by more than 5% versus classical baseline.',
);
assert(
  ltvEkf.metrics.lateralRmse <= classicalEkf.metrics.lateralRmse * 1.05,
  'LTV EKF lateral tracking degraded by more than 5% versus classical EKF baseline.',
);
assert(
  ltvTruth.metrics.headingRmse <= classicalTruth.metrics.headingRmse * 1.05,
  'LTV truth heading tracking degraded by more than 5% versus classical baseline.',
);
assert(
  ltvEkf.metrics.headingRmse <= classicalEkf.metrics.headingRmse * 1.05,
  'LTV EKF heading tracking degraded by more than 5% versus classical EKF baseline.',
);
assert(
  ltvTruth.metrics.controlEffort <= classicalTruth.metrics.controlEffort * 1.20,
  'LTV truth control effort exceeds the +20% research budget.',
);
assert(
  ltvEkf.metrics.controlEffort <= classicalEkf.metrics.controlEffort * 1.20,
  'LTV EKF control effort exceeds the +20% research budget.',
);

for (const [name, value, limit] of [
  ['x RMSE', ltvEkf.metrics.estimateXRmse, 0.08],
  ['y RMSE', ltvEkf.metrics.estimateYRmse, 0.08],
  ['yaw RMSE', ltvEkf.metrics.estimateYawRmse, 0.025],
  ['speed RMSE', ltvEkf.metrics.estimateSpeedRmse, 0.12],
]) {
  assert(Number.isFinite(value) && value < limit, `UGV EKF ${name}=${value} exceeds ${limit}`);
}

assert.deepEqual(
  deterministicTrace(ltvEkf),
  deterministicTrace(ltvEkfRepeat),
  'same-seed UGV LTV+EKF trace must be exactly reproducible',
);
assert.deepEqual(
  ltvEkf.solverRecords.map((record) => [
    record.t,
    record.status,
    record.fallbackUsed,
    record.diagnostics?.iterations,
    record.diagnostics?.feasibilityViolation,
    record.diagnostics?.projectedGradientResidual,
  ]),
  ltvEkfRepeat.solverRecords.map((record) => [
    record.t,
    record.status,
    record.fallbackUsed,
    record.diagnostics?.iterations,
    record.diagnostics?.feasibilityViolation,
    record.diagnostics?.projectedGradientResidual,
  ]),
  'same-seed UGV solver status/diagnostics must be reproducible',
);

const summary = {
  schema: 'mpc-pid-gate5a-ugv/v1',
  timing: {
    plantDt: cfg.dt,
    solveInterval: cfg.ugvLtvMpc.solveInterval,
    predictionDt: cfg.ugvLtvMpc.predictionDt,
    horizon: cfg.ugvLtvMpc.horizon,
    predictionHorizonSeconds: cfg.ugvLtvMpc.horizon * cfg.ugvLtvMpc.predictionDt,
  },
  classicalTruth: classicalTruth.metrics,
  classicalEkf: classicalEkf.metrics,
  ltvTruth: ltvTruth.metrics,
  ltvEkf: ltvEkf.metrics,
  comparison: {
    lateralRatioTruth: ltvTruth.metrics.lateralRmse / classicalTruth.metrics.lateralRmse,
    lateralRatioEkf: ltvEkf.metrics.lateralRmse / classicalEkf.metrics.lateralRmse,
    headingRatioTruth: ltvTruth.metrics.headingRmse / classicalTruth.metrics.headingRmse,
    headingRatioEkf: ltvEkf.metrics.headingRmse / classicalEkf.metrics.headingRmse,
    effortRatioTruth: ltvTruth.metrics.controlEffort / classicalTruth.metrics.controlEffort,
    effortRatioEkf: ltvEkf.metrics.controlEffort / classicalEkf.metrics.controlEffort,
  },
  limitation: 'CI solve latency is simulation evidence only; it is not a target-hardware real-time claim.',
};

console.log('Gate 5A UGV closeout evidence');
console.log(JSON.stringify(summary, null, 2));
console.log('Gate 5A UGV closeout PASS');
