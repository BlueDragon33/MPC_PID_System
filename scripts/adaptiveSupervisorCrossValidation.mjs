import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { createAdaptiveModelSupervisor } from '../src/core/adaptation/adaptiveModelSupervisor.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';

const rms = (values) => values.length ? Math.sqrt(values.reduce((sum, v) => sum + v * v, 0) / values.length) : null;

const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
const trainingCfg = {
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
const training = runSimulation('HYBRID_SAFE', trainingCfg);
const truthCfg = createTruthPlantConfig(trainingCfg);
const truth = { ...truthCfg.plant };
const nominal = { ...trainingCfg.plant };

function train(label, options) {
  const supervisor = createAdaptiveModelSupervisor({
    dt: trainingCfg.dt,
    nominalParameters: nominal,
    windowSize: 2,
    minExcitation: 0,
    excitationHistorySize: 10,
    minInformationRatio: 1e-4,
    minUpdatesBeforePublish: 60,
    publishEveryUpdates: 20,
    maxCovarianceTrace: 8,
    maxRelativePublishStep: 0.03,
    ...options,
  });
  for (let k = 0; k < training.samples.length - 1; k += 1) {
    const a = training.samples[k];
    const b = training.samples[k + 1];
    supervisor.update({
      previousState: { x: a.controllerX, v: a.controllerV },
      nextState: { x: b.controllerX, v: b.controllerV },
      u: a.u,
    });
  }
  return { label, parameters: supervisor.getPublishedModel(), diagnostics: supervisor.getDiagnostics() };
}

const trained = [
  { label: 'nominal', parameters: nominal, diagnostics: null },
  train('PE-only', { validationStride: 0 }),
  train('PE+holdout', {
    validationStride: 4,
    validationBufferSize: 24,
    minValidationSamples: 12,
    minValidationImprovement: 0,
  }),
  { label: 'oracle truth', parameters: truth, diagnostics: null },
];

const nominalModel = createSecondOrderModel(trainingCfg);
function makeEstimator(seed) {
  const sensor = createMeasurementSensor({ C: nominalModel.C, noiseStd: 0.08, seed, bias: 0 });
  const estimator = createAugmentedDisturbanceKalmanFilter({
    A: nominalModel.A,
    B: nominalModel.B,
    E: nominalModel.E,
    C: nominalModel.C,
    processCovariance: [2e-5, 2e-4, 8e-3],
    measurementVariance: 0.08 ** 2,
    initialState: [0, 0, 0],
    initialCovariance: [0.04, 0.08, 0.2],
    disturbanceRetention: 0.90,
  });
  const initialMeasurement = sensor.read({ x: 0, v: 0 });
  estimator.update(initialMeasurement.value);
  return { sensor, estimator };
}

function validationExcitation(t) {
  return 0.85 * Math.sin(0.47 * t + 0.31)
    + 0.60 * Math.sin(2.11 * t + 0.77)
    + 0.24 * (Math.sin(0.19 * t + 0.52) >= 0 ? 1 : -1);
}

function collectValidation(seed) {
  const { sensor, estimator } = makeEstimator(seed);
  let estimate = estimator.getState();
  let truthState = { x: 0, v: 0 };
  const data = [];
  const steps = Math.floor(18 / trainingCfg.dt);
  for (let k = 0; k <= steps; k += 1) {
    const t = k * trainingCfg.dt;
    const u = validationExcitation(t);
    data.push({ t, u, estimateX: estimate.x, estimateV: estimate.v, truthX: truthState.x, truthV: truthState.v });
    if (k === steps) break;
    const disturbance = t >= 6.0 && t < 7.1 ? 0.45 : 0;
    const nextTruth = stepSecondOrderPlant(truthState, u, disturbance, truthCfg);
    const measurement = sensor.read(nextTruth);
    estimate = estimator.step(u, measurement.value);
    truthState = nextTruth;
  }
  return data;
}

function modelCfg(parameters) {
  return { ...trainingCfg, truthPlant: { ...trainingCfg.truthPlant, enabled: false }, plant: { ...parameters } };
}

function validate(parameters, data, horizon) {
  const cfg = modelCfg(parameters);
  const xErrors = [];
  const vErrors = [];
  for (let k = 0; k + horizon <= data.length - 1; k += 1) {
    let predicted = { x: data[k].estimateX, v: data[k].estimateV };
    for (let j = 0; j < horizon; j += 1) predicted = stepSecondOrderPlant(predicted, data[k + j].u, 0, cfg);
    const target = data[k + horizon];
    xErrors.push(predicted.x - target.truthX);
    vErrors.push(predicted.v - target.truthV);
  }
  return { xRmse: rms(xErrors), vRmse: rms(vErrors) };
}

const horizons = [1, 12, 35];
const seeds = [20260916, 20260917, 20260918];
const rows = [];
for (const candidate of trained) {
  const aggregate = Object.fromEntries(horizons.map((h) => [h, { x: [], v: [] }]));
  for (const seed of seeds) {
    const data = collectValidation(seed);
    for (const horizon of horizons) {
      const metric = validate(candidate.parameters, data, horizon);
      aggregate[horizon].x.push(metric.xRmse);
      aggregate[horizon].v.push(metric.vRmse);
    }
  }
  rows.push({
    ...candidate,
    byHorizon: Object.fromEntries(horizons.map((h) => [h, {
      xRmse: aggregate[h].x.reduce((a, b) => a + b, 0) / aggregate[h].x.length,
      vRmse: aggregate[h].v.reduce((a, b) => a + b, 0) / aggregate[h].v.length,
    }])),
  });
}

console.log('Adaptive supervisor cross-validation across unseen seeds/excitation');
console.table(rows.map((row) => ({
  model: row.label,
  k: row.parameters.stiffness.toFixed(4),
  c: row.parameters.damping.toFixed(4),
  g: row.parameters.gain.toFixed(4),
  'x H1': row.byHorizon[1].xRmse.toFixed(4),
  'v H1': row.byHorizon[1].vRmse.toFixed(4),
  'x H12': row.byHorizon[12].xRmse.toFixed(4),
  'v H12': row.byHorizon[12].vRmse.toFixed(4),
  'x H35': row.byHorizon[35].xRmse.toFixed(4),
  'v H35': row.byHorizon[35].vRmse.toFixed(4),
  publishes: row.diagnostics?.publishCount ?? '—',
})));
const nominalRow = rows.find((row) => row.label === 'nominal');
for (const row of rows.filter((item) => item.label !== 'nominal')) {
  console.log(`${row.label}: H35 x ratio=${(row.byHorizon[35].xRmse / nominalRow.byHorizon[35].xRmse).toFixed(3)}, v ratio=${(row.byHorizon[35].vRmse / nominalRow.byHorizon[35].vRmse).toFixed(3)}`);
}
