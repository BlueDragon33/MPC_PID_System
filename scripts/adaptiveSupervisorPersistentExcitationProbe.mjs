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

function evaluate(historySize, minInformationRatio) {
  const supervisor = createAdaptiveModelSupervisor({
    dt: config.dt,
    nominalParameters: config.plant,
    windowSize: 2,
    minExcitation: 0,
    excitationHistorySize: historySize,
    minInformationRatio,
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
    historySize,
    minInformationRatio,
    diagnostics,
    candidate,
    published,
    candidateError: meanParameterError(candidate, truth),
    publishedError: meanParameterError(published, truth),
  };
}

const rows = [];
for (const historySize of [10, 20, 30, 50]) {
  for (const threshold of [0, 1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2]) {
    rows.push(evaluate(historySize, threshold));
  }
}

console.log('Adaptive supervisor persistent-excitation information sweep');
console.log(`Nominal mean parameter error: ${(100 * nominalError).toFixed(2)}%`);
console.table(rows.map((entry) => ({
  history: entry.historySize,
  threshold: entry.minInformationRatio.toExponential(1),
  accepted: entry.diagnostics.acceptedWindows,
  fillReject: entry.diagnostics.rejectedInformationFilling,
  peReject: entry.diagnostics.rejectedLowInformation,
  publishes: entry.diagnostics.publishCount,
  finalInfo: entry.diagnostics.information.normalizedDeterminant.toExponential(3),
  covariance: entry.diagnostics.rls?.covarianceTrace?.toFixed(4) ?? 'n/a',
  k: entry.published.stiffness.toFixed(4),
  c: entry.published.damping.toFixed(4),
  g: entry.published.gain.toFixed(4),
  'candidate err %': (100 * entry.candidateError).toFixed(2),
  'published err %': (100 * entry.publishedError).toFixed(2),
})));

const viable = rows
  .filter((entry) => entry.diagnostics.acceptedWindows >= 60 && entry.diagnostics.publishCount > 0)
  .sort((a, b) => a.publishedError - b.publishedError);
const best = viable[0];
if (best) {
  console.log('Best viable PE gate');
  console.table([{
    history: best.historySize,
    threshold: best.minInformationRatio.toExponential(2),
    accepted: best.diagnostics.acceptedWindows,
    peReject: best.diagnostics.rejectedLowInformation,
    publishes: best.diagnostics.publishCount,
    k: best.published.stiffness.toFixed(4),
    c: best.published.damping.toFixed(4),
    g: best.published.gain.toFixed(4),
    'published err %': (100 * best.publishedError).toFixed(2),
    'vs nominal ratio': (best.publishedError / nominalError).toFixed(3),
  }]);
} else {
  console.log('No viable persistent-excitation gate published a model in this trajectory.');
}
