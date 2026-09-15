import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';
import { createAdaptiveModelSupervisor } from '../src/core/adaptation/adaptiveModelSupervisor.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}
const relativeError = (estimate, truth) => Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);
const meanParameterError = (p, truth) => (
  relativeError(p.stiffness, truth.stiffness)
  + relativeError(p.damping, truth.damping)
  + relativeError(p.gain, truth.gain)
) / 3;

function excitation(t) {
  return 1.2 * Math.sin(0.73 * t) + 0.8 * Math.sin(1.93 * t) + 0.35 * (Math.sin(0.31 * t) >= 0 ? 1 : -1);
}

const cfg = {
  ...defaultConfig,
  truthPlant: { enabled: true, stiffnessScale: 1.18, dampingScale: 0.78, gainScale: 0.90 },
};
const nominalModel = createSecondOrderModel(cfg);
const truthCfg = createTruthPlantConfig(cfg);
const truth = truthCfg.plant;
const sensor = createMeasurementSensor({ C: nominalModel.C, noiseStd: 0.03, seed: 20260915, bias: 0 });
const estimator = createAugmentedDisturbanceKalmanFilter({
  A: nominalModel.A,
  B: nominalModel.B,
  E: nominalModel.E,
  C: nominalModel.C,
  processCovariance: [2e-5, 2e-4, 8e-3],
  measurementVariance: 0.03 ** 2,
  initialState: [0, 0, 0],
  initialCovariance: [0.04, 0.08, 0.2],
  disturbanceRetention: 0.90,
});
const supervisor = createAdaptiveModelSupervisor({
  dt: cfg.dt,
  nominalParameters: cfg.plant,
  windowSize: 2,
  minExcitation: 0.08,
  minUpdatesBeforePublish: 80,
  publishEveryUpdates: 20,
  maxCovarianceTrace: 5,
  maxRelativePublishStep: 0.03,
});

let truthState = { x: 0, v: 0 };
const firstMeasurement = sensor.read(truthState);
let estimate = estimator.update(firstMeasurement.value);
const steps = Math.floor(24 / cfg.dt);
for (let k = 0; k < steps; k += 1) {
  const t = k * cfg.dt;
  const u = excitation(t);
  const previousEstimate = { x: estimate.x, v: estimate.v };
  const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
  const measurement = sensor.read(nextTruth);
  estimate = estimator.step(u, measurement.value);
  supervisor.update({
    previousState: previousEstimate,
    nextState: { x: estimate.x, v: estimate.v },
    u,
  });
  truthState = nextTruth;
}

const diagnostics = supervisor.getDiagnostics();
const candidate = supervisor.getCandidate();
const published = supervisor.getPublishedModel();
const nominalError = meanParameterError(cfg.plant, truth);
const candidateError = meanParameterError(candidate, truth);
const publishedError = meanParameterError(published, truth);

assert(diagnostics.shadowMode === true, 'Adaptive supervisor must remain shadow-only at this gate.');
assert(diagnostics.acceptedWindows > 400, `Too few accepted identification windows: ${diagnostics.acceptedWindows}`);
assert(diagnostics.publishCount > 0, 'Adaptive supervisor never published a bounded shadow model.');
assert(Object.values(candidate).every(Number.isFinite), 'Adaptive candidate contains non-finite parameters.');
assert(Object.values(published).every(Number.isFinite), 'Published shadow model contains non-finite parameters.');
assert(candidateError < 0.04, `Candidate mean parameter error too high: ${(100 * candidateError).toFixed(2)}%`);
assert(publishedError < nominalError * 0.35, `Published shadow model did not improve enough over nominal: ratio=${(publishedError / nominalError).toFixed(3)}`);
assert(relativeError(published.gain, truth.gain) < 0.02, 'Published gain estimate is outside the expected held-out identification region.');
assert(relativeError(published.damping, truth.damping) < 0.03, 'Published damping estimate is outside the expected held-out identification region.');

console.log('Adaptive model supervisor shadow smoke PASS');
console.table([
  {
    model: 'nominal',
    stiffness: cfg.plant.stiffness.toFixed(4),
    damping: cfg.plant.damping.toFixed(4),
    gain: cfg.plant.gain.toFixed(4),
    'mean err %': (100 * nominalError).toFixed(2),
  },
  {
    model: 'candidate',
    stiffness: candidate.stiffness.toFixed(4),
    damping: candidate.damping.toFixed(4),
    gain: candidate.gain.toFixed(4),
    'mean err %': (100 * candidateError).toFixed(2),
  },
  {
    model: 'published-shadow',
    stiffness: published.stiffness.toFixed(4),
    damping: published.damping.toFixed(4),
    gain: published.gain.toFixed(4),
    'mean err %': (100 * publishedError).toFixed(2),
  },
]);
console.log(`AdaptiveSupervisor: accepted=${diagnostics.acceptedWindows}, rejectedLowExcitation=${diagnostics.rejectedLowExcitation}, publishes=${diagnostics.publishCount}, covarianceTrace=${diagnostics.rls?.covarianceTrace?.toFixed(4) ?? 'n/a'}`);
