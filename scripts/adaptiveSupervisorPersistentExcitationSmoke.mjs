import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { createAdaptiveModelSupervisor } from '../src/core/adaptation/adaptiveModelSupervisor.js';
import { createTruthPlantConfig } from '../src/core/models/secondOrderPlant.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}
const relativeError = (estimate, truth) => Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);
const meanParameterError = (p, truth) => (
  relativeError(p.stiffness, truth.stiffness)
  + relativeError(p.damping, truth.damping)
  + relativeError(p.gain, truth.gain)
) / 3;

const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
const config = {
  ...preset,
  duration: 12,
  disturbance: { ...preset.disturbance, enabled: true, start: 3.0, duration: 1.2, amplitude: 0.8 },
  mpc: {
    ...preset.mpc,
    horizon: 12,
    qpIterations: Math.max(100, preset.mpc.qpIterations),
    qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
    qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
  },
};
const result = runSimulation('HYBRID_SAFE', config);
const truth = createTruthPlantConfig(config).plant;
const nominalError = meanParameterError(config.plant, truth);

const supervisor = createAdaptiveModelSupervisor({
  dt: config.dt,
  nominalParameters: config.plant,
  windowSize: 2,
  minExcitation: 0,
  excitationHistorySize: 10,
  minInformationRatio: 1e-4,
  minUpdatesBeforePublish: 60,
  publishEveryUpdates: 20,
  maxCovarianceTrace: 8,
  maxRelativePublishStep: 0.03,
});

for (let k = 0; k < result.samples.length - 1; k += 1) {
  const a = result.samples[k];
  const b = result.samples[k + 1];
  supervisor.update({
    previousState: { x: a.controllerX, v: a.controllerV },
    nextState: { x: b.controllerX, v: b.controllerV },
    u: a.u,
  });
}

const diagnostics = supervisor.getDiagnostics();
const published = supervisor.getPublishedModel();
const candidate = supervisor.getCandidate();
const publishedError = meanParameterError(published, truth);
const candidateError = meanParameterError(candidate, truth);

assert(diagnostics.shadowMode === true, 'PE supervisor must remain shadow-only.');
assert(diagnostics.acceptedWindows >= 220 && diagnostics.acceptedWindows <= 260,
  `Unexpected accepted-window count: ${diagnostics.acceptedWindows}`);
assert(diagnostics.rejectedLowInformation >= 40,
  `Persistent-excitation gate rejected too few windows: ${diagnostics.rejectedLowInformation}`);
assert(diagnostics.publishCount >= 8, `Too few bounded shadow publishes: ${diagnostics.publishCount}`);
assert(publishedError < 0.07, `PE published model error too high: ${(100 * publishedError).toFixed(2)}%`);
assert(publishedError < nominalError * 0.40,
  `PE published model did not improve enough over nominal: ratio=${(publishedError / nominalError).toFixed(3)}`);
assert(candidateError > publishedError,
  'Regression expected bounded published model to remain safer than drifting final candidate on this closed-loop trajectory.');

console.log('Adaptive supervisor persistent-excitation smoke PASS');
console.table([
  { model: 'nominal', 'mean err %': (100 * nominalError).toFixed(2) },
  { model: 'candidate', 'mean err %': (100 * candidateError).toFixed(2) },
  { model: 'published-PE', 'mean err %': (100 * publishedError).toFixed(2) },
]);
console.log(`PE: accepted=${diagnostics.acceptedWindows}, rejected=${diagnostics.rejectedLowInformation}, publishes=${diagnostics.publishCount}, info=${diagnostics.information.normalizedDeterminant.toExponential(3)}`);
