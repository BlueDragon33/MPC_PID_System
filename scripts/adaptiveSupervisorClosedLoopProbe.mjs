import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { createAdaptiveModelSupervisor } from '../src/core/adaptation/adaptiveModelSupervisor.js';
import { createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';

const rms = (values) => values.length ? Math.sqrt(values.reduce((s, v) => s + v * v, 0) / values.length) : null;
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

function evaluateThreshold(minExcitation) {
  const supervisor = createAdaptiveModelSupervisor({
    dt: config.dt,
    nominalParameters: config.plant,
    windowSize: 2,
    minExcitation,
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
    minExcitation,
    diagnostics,
    candidate,
    published,
    candidateError: meanParameterError(candidate, truth),
    publishedError: meanParameterError(published, truth),
  };
}

const thresholds = [0, 0.02, 0.05, 0.08, 0.12, 0.2].map(evaluateThreshold);
console.log('Adaptive supervisor excitation probe on HYBRID_SAFE closed-loop trajectory');
console.log(`Truth: k=${truth.stiffness.toFixed(4)}, c=${truth.damping.toFixed(4)}, g=${truth.gain.toFixed(4)}`);
console.table(thresholds.map((entry) => ({
  minExcitation: entry.minExcitation.toFixed(2),
  accepted: entry.diagnostics.acceptedWindows,
  rejected: entry.diagnostics.rejectedLowExcitation,
  publishes: entry.diagnostics.publishCount,
  covarianceTrace: entry.diagnostics.rls?.covarianceTrace?.toFixed(4) ?? 'n/a',
  k: entry.published.stiffness.toFixed(4),
  c: entry.published.damping.toFixed(4),
  g: entry.published.gain.toFixed(4),
  'candidate err %': (100 * entry.candidateError).toFixed(2),
  'published err %': (100 * entry.publishedError).toFixed(2),
})));

function predictionRmse(parameters, startTime = 6, horizon = 12) {
  const modelCfg = { ...config, truthPlant: { ...config.truthPlant, enabled: false }, plant: { ...parameters } };
  const xErrors = [];
  const vErrors = [];
  const startIndex = result.samples.findIndex((s) => s.t >= startTime);
  for (let k = Math.max(0, startIndex); k + horizon < result.samples.length; k += 1) {
    let predicted = { x: result.samples[k].controllerX, v: result.samples[k].controllerV };
    for (let j = 0; j < horizon; j += 1) {
      const sample = result.samples[k + j];
      const d = sample.disturbancePredictionEnabled ? sample.estimateD ?? 0 : 0;
      predicted = stepSecondOrderPlant(predicted, sample.u, d, modelCfg);
    }
    const target = result.samples[k + horizon];
    xErrors.push(predicted.x - target.x);
    vErrors.push(predicted.v - target.v);
  }
  return { xRmse: rms(xErrors), vRmse: rms(vErrors) };
}

const selected = thresholds.find((entry) => entry.minExcitation === 0.08);
const nominalPrediction = predictionRmse(config.plant);
const shadowPrediction = predictionRmse(selected.published);
const oraclePrediction = predictionRmse(truth);
console.log('Post-learning closed-loop shadow prediction (H=12)');
console.table([
  { model: 'nominal', xRMSE: nominalPrediction.xRmse.toFixed(4), vRMSE: nominalPrediction.vRmse.toFixed(4) },
  { model: 'published shadow', xRMSE: shadowPrediction.xRmse.toFixed(4), vRMSE: shadowPrediction.vRmse.toFixed(4) },
  { model: 'oracle truth', xRMSE: oraclePrediction.xRmse.toFixed(4), vRMSE: oraclePrediction.vRmse.toFixed(4) },
]);
console.log(`Published/nominal prediction ratios: x=${(shadowPrediction.xRmse / nominalPrediction.xRmse).toFixed(3)}, v=${(shadowPrediction.vRmse / nominalPrediction.vRmse).toFixed(3)}`);
