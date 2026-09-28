import assert from 'node:assert/strict';

import {
  assertControllerComparisonContract,
  assertSimulationResultContract,
} from '../src/contracts/workbenchContracts.js';
import { createEstimatorRuntime } from '../src/core/orchestration/estimatorRuntime.js';
import { createMpcPlanRuntime } from '../src/core/orchestration/mpcPlanRuntime.js';
import {
  defaultConfig,
  mergeSimulationConfig,
} from '../src/core/orchestration/simulationConfig.js';
import { createSecondOrderModel } from '../src/core/models/secondOrderPlant.js';
import { compareControllers, runSimulation } from '../src/core/simulator.js';

const originalKp = defaultConfig.pid.kp;
const merged = mergeSimulationConfig({
  duration: 0.16,
  pid: { kp: 7.7 },
  estimation: {
    enabled: true,
    measurementNoiseStd: 0.1,
    seed: 4242,
  },
  disturbance: { enabled: false },
});

assert.equal(merged.pid.kp, 7.7);
assert.equal(merged.pid.ki, defaultConfig.pid.ki, 'nested defaults must be preserved');
assert.equal(defaultConfig.pid.kp, originalKp, 'merge must not mutate defaultConfig');

const model = createSecondOrderModel(merged);
const initialState = { x: 0.25, v: -0.1 };
const estimatorA = createEstimatorRuntime({ cfg: merged, model, initialState });
const estimatorB = createEstimatorRuntime({ cfg: merged, model, initialState });

assert.equal(estimatorA.flags.estimationEnabled, true);
assert.deepEqual(
  estimatorA.current().measurementSample,
  estimatorB.current().measurementSample,
  'same seed must produce deterministic initial measurement',
);
assert(Number.isFinite(estimatorA.current().controllerState.x));
assert(Number.isFinite(estimatorA.current().controllerState.v));
assert(Array.isArray(estimatorA.current().covariance));

const truthCfg = mergeSimulationConfig({
  estimation: { enabled: false },
  disturbance: { enabled: false },
});
const truthRuntime = createEstimatorRuntime({
  cfg: truthCfg,
  model: createSecondOrderModel(truthCfg),
  initialState,
});
assert.deepEqual(truthRuntime.current().controllerState, initialState);
assert.equal(truthRuntime.current().covariance, null);
assert.equal(truthRuntime.current().disturbanceEstimate, 0);

const planRuntime = createMpcPlanRuntime();
const planSolution = planRuntime.solve({
  state: { x: 0, v: 0 },
  target: 1,
  previousU: 0,
  cfg: mergeSimulationConfig({
    mpc: {
      horizon: 8,
      qpIterations: 40,
      qpProjectionCycles: 8,
    },
    disturbance: { enabled: false },
  }),
});
assert(Number.isFinite(planSolution.u));
assert.equal(planRuntime.getSolverRecords().length, 1);
if (!planSolution.fallbackUsed) {
  assert(Array.isArray(planRuntime.getContinuationSequence()));
  assert.equal(planRuntime.getContinuationSequence().length, planSolution.sequence.length);
}
planRuntime.shiftPlan();
assert.equal(planRuntime.getSolverRecords().length, 1, 'plan shifting must not create solver records');

const simulation = runSimulation('PID', {
  duration: 0.12,
  disturbance: { enabled: false },
});
assertSimulationResultContract(simulation);
assert.throws(
  () => assertSimulationResultContract({ samples: [] }),
  /metrics must be an object/,
);

const comparison = compareControllers({
  duration: 0.12,
  disturbance: { enabled: false },
  mpc: { horizon: 8, qpIterations: 40, qpProjectionCycles: 8 },
});
assertControllerComparisonContract(comparison);
assert.equal(comparison.length, 4);

console.log('orchestration contract smoke: PASS', {
  deterministicSeed: merged.estimation.seed,
  solverRecords: planRuntime.getSolverRecords().length,
  comparisonModes: comparison.map((item) => item.mode),
});
