import assert from 'node:assert/strict';

import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { SOLVER_BACKENDS } from '../src/core/solvers/index.js';

function scenario(presetId, overrides = {}) {
  const preset = applyExperimentPreset(defaultConfig, presetId);
  return {
    ...preset,
    duration: 4.2,
    ...overrides,
    truthPlant: {
      ...preset.truthPlant,
      ...(overrides.truthPlant || {}),
    },
    estimation: {
      ...preset.estimation,
      ...(overrides.estimation || {}),
    },
    disturbance: {
      ...preset.disturbance,
      enabled: true,
      start: 1.15,
      duration: 1.1,
      amplitude: 1.0,
      ...(overrides.disturbance || {}),
    },
    mpc: {
      ...preset.mpc,
      solver: SOLVER_BACKENDS.CONSTRAINED_QP,
      horizon: 12,
      qpIterations: Math.max(100, preset.mpc.qpIterations),
      qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
      qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
      ...(overrides.mpc || {}),
    },
  };
}

function strictSafe(result, label) {
  assert.equal(result.metrics.fallbackCount, 0, `${label}: fallbackCount=${result.metrics.fallbackCount}`);
  assert.equal(result.metrics.infeasibleCount, 0, `${label}: infeasibleCount=${result.metrics.infeasibleCount}`);
  assert.equal(result.metrics.timeoutCount, 0, `${label}: timeoutCount=${result.metrics.timeoutCount}`);
  assert.equal(result.metrics.numericalFailureCount, 0, `${label}: numericalFailureCount=${result.metrics.numericalFailureCount}`);
  assert((result.metrics.convergenceRate ?? 100) >= 99.9, `${label}: convergence=${result.metrics.convergenceRate}`);
  assert(result.metrics.maxActualSafetyViolation <= 1e-9, `${label}: plant safety violation=${result.metrics.maxActualSafetyViolation}`);
}

function predictionStats(result) {
  const predictionErrors = result.samples.map((sample) => sample.predictionError).filter(Number.isFinite);
  return {
    predictionTriggers: result.samples.filter(
      (sample) => sample.triggered && sample.triggerReason === 'prediction-error',
    ).length,
    avgPredictionError: predictionErrors.reduce((sum, value) => sum + value, 0)
      / Math.max(1, predictionErrors.length),
  };
}

function deterministicTrace(result) {
  return result.samples.map((sample) => [
    sample.measurement,
    sample.controllerX,
    sample.controllerV,
    sample.estimateD,
    sample.u,
    sample.triggered,
    sample.triggerReason,
  ]);
}

const stateOnlyCfg = scenario('model-mismatch');
const observerMonitorOffCfg = scenario('mismatch-observer', {
  estimation: {
    disturbancePredictionEnabled: false,
    mpcDisturbanceCompensationEnabled: false,
  },
});
const observerMonitorOnCfg = scenario('mismatch-observer', {
  estimation: {
    disturbancePredictionEnabled: true,
    mpcDisturbanceCompensationEnabled: false,
  },
});
const observerMpcCompCfg = scenario('mismatch-observer', {
  estimation: {
    disturbancePredictionEnabled: true,
    mpcDisturbanceCompensationEnabled: true,
  },
});

const stateOnly = runSimulation('HYBRID_SAFE', stateOnlyCfg);
const observerOff = runSimulation('HYBRID_SAFE', observerMonitorOffCfg);
const observerOn = runSimulation('HYBRID_SAFE', observerMonitorOnCfg);
const observerOnRepeat = runSimulation('HYBRID_SAFE', observerMonitorOnCfg);
const mpcComp = runSimulation('HYBRID_SAFE', observerMpcCompCfg);

for (const [label, result] of [
  ['2-state mismatch baseline', stateOnly],
  ['observer monitor OFF', observerOff],
  ['observer monitor ON', observerOn],
  ['MPC affine compensation', mpcComp],
]) {
  strictSafe(result, label);
  assert.equal(result.metrics.truthPlantMismatchEnabled, true, `${label}: truth/model mismatch is not active`);
}

assert.equal(stateOnly.metrics.disturbanceEstimateEnabled, false);
assert.equal(observerOn.metrics.disturbanceEstimateEnabled, true);
assert(Number.isFinite(observerOn.metrics.disturbanceEstimateRmse));
assert(
  observerOn.metrics.disturbanceEstimateRmse < 0.55,
  `observer disturbance RMSE too high: ${observerOn.metrics.disturbanceEstimateRmse}`,
);
assert(Number.isFinite(observerOn.metrics.estimatePositionRmse));
assert(Number.isFinite(observerOn.metrics.estimateVelocityRmse));
assert(
  observerOn.metrics.estimatePositionRmse <= stateOnly.metrics.estimatePositionRmse * 1.35,
  'augmented observer degraded position RMSE excessively',
);
assert(
  observerOn.metrics.estimateVelocityRmse <= stateOnly.metrics.estimateVelocityRmse * 1.35,
  'augmented observer degraded velocity RMSE excessively',
);

assert.deepEqual(
  deterministicTrace(observerOn),
  deterministicTrace(observerOnRepeat),
  'same seed/config must reproduce measurement, estimate, command and trigger traces exactly',
);

const offStats = predictionStats(observerOff);
const onStats = predictionStats(observerOn);
assert(
  observerOn.metrics.solveCount <= observerOff.metrics.solveCount,
  `d-hat Event Monitor increased solve count: ${observerOff.metrics.solveCount} -> ${observerOn.metrics.solveCount}`,
);
assert(
  onStats.predictionTriggers <= offStats.predictionTriggers,
  `d-hat Event Monitor increased prediction triggers: ${offStats.predictionTriggers} -> ${onStats.predictionTriggers}`,
);
assert(
  observerOn.metrics.iae <= observerOff.metrics.iae * 1.01,
  `d-hat Event Monitor degraded IAE >1%: ${observerOff.metrics.iae} -> ${observerOn.metrics.iae}`,
);
assert(
  onStats.avgPredictionError <= offStats.avgPredictionError * 1.02,
  `d-hat Event Monitor degraded average prediction error: ${offStats.avgPredictionError} -> ${onStats.avgPredictionError}`,
);

assert.equal(mpcComp.metrics.mpcDisturbanceCompensationEnabled, true);
assert(
  mpcComp.solverRecords.some((record) => record.diagnostics?.disturbanceCompensationEnabled),
  'affine MPC compensation did not reach the QP solver diagnostics',
);

// Gate-4B policy: Event Monitor compensation is promoted in mismatch-observer.
// Affine MPC horizon compensation remains opt-in until a later multi-plant gate
// demonstrates a robust quality/compute benefit, even though feasibility/safety pass here.
const promotedPreset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
assert.equal(promotedPreset.estimation.disturbancePredictionEnabled, true);
assert.equal(promotedPreset.estimation.mpcDisturbanceCompensationEnabled, false);

const summary = {
  schema: 'mpc-pid-gate4b-closeout/v1',
  stateOnly: {
    iae: stateOnly.metrics.iae,
    solves: stateOnly.metrics.solveCount,
    xRmse: stateOnly.metrics.estimatePositionRmse,
    vRmse: stateOnly.metrics.estimateVelocityRmse,
  },
  observer: {
    iae: observerOn.metrics.iae,
    solves: observerOn.metrics.solveCount,
    xRmse: observerOn.metrics.estimatePositionRmse,
    vRmse: observerOn.metrics.estimateVelocityRmse,
    dRmse: observerOn.metrics.disturbanceEstimateRmse,
    activeDRmse: observerOn.metrics.activeDisturbanceEstimateRmse,
    retention: promotedPreset.estimation.disturbanceRetention,
  },
  eventMonitor: {
    offSolves: observerOff.metrics.solveCount,
    onSolves: observerOn.metrics.solveCount,
    offPredictionTriggers: offStats.predictionTriggers,
    onPredictionTriggers: onStats.predictionTriggers,
    offAvgPredictionError: offStats.avgPredictionError,
    onAvgPredictionError: onStats.avgPredictionError,
    iaeRatio: observerOn.metrics.iae / Math.max(1e-12, observerOff.metrics.iae),
  },
  affineMpcOptIn: {
    iae: mpcComp.metrics.iae,
    solves: mpcComp.metrics.solveCount,
    convergenceRate: mpcComp.metrics.convergenceRate,
    fallbackCount: mpcComp.metrics.fallbackCount,
    plantViolation: mpcComp.metrics.maxActualSafetyViolation,
  },
};

console.log('Gate 4B closeout evidence');
console.log(JSON.stringify(summary, null, 2));
console.log('Gate 4B closeout PASS');
