import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { createAdaptiveModelSupervisor } from '../src/core/adaptation/adaptiveModelSupervisor.js';
import { createTruthPlantConfig } from '../src/core/models/secondOrderPlant.js';

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

function evaluate(validationStride, validationBufferSize, minValidationImprovement) {
  const supervisor = createAdaptiveModelSupervisor({
    dt: config.dt,
    nominalParameters: config.plant,
    windowSize: 2,
    minExcitation: 0,
    excitationHistorySize: 10,
    minInformationRatio: 1e-4,
    validationStride,
    validationBufferSize,
    minValidationSamples: Math.min(12, validationBufferSize),
    minValidationImprovement,
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
  const candidate = supervisor.getCandidate();
  const published = supervisor.getPublishedModel();
  return {
    validationStride,
    validationBufferSize,
    minValidationImprovement,
    diagnostics,
    candidate,
    published,
    candidateError: meanParameterError(candidate, truth),
    publishedError: meanParameterError(published, truth),
  };
}

const rows = [];
for (const stride of [3, 4, 5, 6]) {
  for (const bufferSize of [12, 24, 40]) {
    for (const improvement of [0, 0.01, 0.03, 0.05]) rows.push(evaluate(stride, bufferSize, improvement));
  }
}

console.log('Adaptive supervisor holdout validation-gate sweep');
console.log(`Nominal mean parameter error: ${(100 * nominalError).toFixed(2)}%`);
console.table(rows.map((entry) => ({
  stride: entry.validationStride,
  buffer: entry.validationBufferSize,
  improve: entry.minValidationImprovement.toFixed(2),
  accepted: entry.diagnostics.acceptedWindows,
  holdout: entry.diagnostics.validationWindows,
  peReject: entry.diagnostics.rejectedLowInformation,
  valReject: entry.diagnostics.rejectedValidation,
  publishes: entry.diagnostics.publishCount,
  candidateRMSE: entry.diagnostics.validation.candidateRmse?.toFixed(4) ?? 'n/a',
  publishedRMSE: entry.diagnostics.validation.publishedRmse?.toFixed(4) ?? 'n/a',
  valGain: entry.diagnostics.validation.improvementRatio?.toFixed(3) ?? 'n/a',
  k: entry.published.stiffness.toFixed(4),
  c: entry.published.damping.toFixed(4),
  g: entry.published.gain.toFixed(4),
  'candidate err %': (100 * entry.candidateError).toFixed(2),
  'published err %': (100 * entry.publishedError).toFixed(2),
})));

const viable = rows
  .filter((entry) => entry.diagnostics.publishCount > 0 && entry.diagnostics.validation.samples >= 12)
  .sort((a, b) => a.publishedError - b.publishedError);
const best = viable[0];
if (best) {
  console.log('Best viable holdout validation gate');
  console.table([{
    stride: best.validationStride,
    buffer: best.validationBufferSize,
    improve: best.minValidationImprovement.toFixed(2),
    accepted: best.diagnostics.acceptedWindows,
    holdout: best.diagnostics.validationWindows,
    valReject: best.diagnostics.rejectedValidation,
    publishes: best.diagnostics.publishCount,
    candidateRMSE: best.diagnostics.validation.candidateRmse?.toFixed(4) ?? 'n/a',
    publishedRMSE: best.diagnostics.validation.publishedRmse?.toFixed(4) ?? 'n/a',
    k: best.published.stiffness.toFixed(4),
    c: best.published.damping.toFixed(4),
    g: best.published.gain.toFixed(4),
    'candidate err %': (100 * best.candidateError).toFixed(2),
    'published err %': (100 * best.publishedError).toFixed(2),
    'vs nominal ratio': (best.publishedError / nominalError).toFixed(3),
  }]);
} else {
  console.log('No validation-gated configuration published a model.');
}
