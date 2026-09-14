import { defaultConfig, runSimulation } from '../src/core/simulator.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function makeConfig(seed) {
  return {
    ...defaultConfig,
    duration: 2.0,
    estimation: {
      ...defaultConfig.estimation,
      enabled: true,
      measurementNoiseStd: 0.1,
      seed,
      processPositionVariance: 2e-5,
      processVelocityVariance: 2e-4,
    },
    mpc: {
      ...defaultConfig.mpc,
      horizon: 16,
      qpIterations: 50,
      stateConstraintsEnabled: false,
      outputConstraintsEnabled: false,
    },
    disturbance: {
      enabled: true,
      start: 0.65,
      duration: 0.45,
      amplitude: 1.1,
    },
  };
}

const config = makeConfig(424242);
const first = runSimulation('HYBRID_SAFE', config);
const second = runSimulation('HYBRID_SAFE', makeConfig(424242));
const differentSeed = runSimulation('HYBRID_SAFE', makeConfig(424243));

assert(first.metrics.estimationEnabled, 'Closed-loop estimation metrics must be enabled.');
assert(first.metrics.measurementRmse > 0.05, `Measurement noise is unexpectedly small: ${first.metrics.measurementRmse}`);
assert(first.metrics.estimatePositionRmse < first.metrics.measurementRmse, `Kalman estimate did not improve position RMSE: measurement=${first.metrics.measurementRmse}, estimate=${first.metrics.estimatePositionRmse}`);
assert(Number.isFinite(first.metrics.estimateVelocityRmse), 'Velocity estimation RMSE must be finite.');
assert(first.metrics.numericalFailureCount === 0, 'Estimated-state closed loop must not produce solver numerical failures.');
assert(first.samples.some((sample) => Math.abs(sample.controllerX - sample.x) > 1e-4), 'Controller state appears to be leaking ground-truth position.');
assert(first.samples.some((sample) => Math.abs(sample.controllerV - sample.v) > 1e-4), 'Controller state appears to be leaking ground-truth velocity.');

assert(first.samples.length === second.samples.length, 'Repeat run changed sample count.');
for (let i = 0; i < first.samples.length; i += 1) {
  assert(first.samples[i].measurement === second.samples[i].measurement, `Seeded measurement is not deterministic at sample ${i}.`);
  assert(first.samples[i].controllerX === second.samples[i].controllerX, `Seeded estimate is not deterministic at sample ${i}.`);
  assert(first.samples[i].u === second.samples[i].u, `Seeded closed-loop control is not deterministic at sample ${i}.`);
}

const measurementChanged = first.samples.some((sample, i) => sample.measurement !== differentSeed.samples[i]?.measurement);
assert(measurementChanged, 'Changing the measurement seed should change the noisy sensor sequence.');

console.log('Closed-loop estimated-state smoke PASS');
console.log(`ClosedLoopEstimation: measurementRMSE=${first.metrics.measurementRmse.toFixed(5)}, positionRMSE=${first.metrics.estimatePositionRmse.toFixed(5)}, velocityRMSE=${first.metrics.estimateVelocityRmse.toFixed(5)}, solves=${first.metrics.solveCount}`);
