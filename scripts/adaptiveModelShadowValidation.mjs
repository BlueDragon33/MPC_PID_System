import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';
import { createRecursiveLeastSquaresPlantEstimator } from '../src/core/estimation/recursiveLeastSquaresPlantEstimator.js';

const rms = (values) => values.length ? Math.sqrt(values.reduce((s, v) => s + v * v, 0) / values.length) : null;

const cfg = {
  ...defaultConfig,
  truthPlant: { enabled: true, stiffnessScale: 1.18, dampingScale: 0.78, gainScale: 0.90 },
};
const nominal = { ...cfg.plant };
const truthCfg = createTruthPlantConfig(cfg);
const truth = { ...truthCfg.plant };
const nominalModel = createSecondOrderModel(cfg);

function identificationExcitation(t) {
  return 1.2 * Math.sin(0.73 * t) + 0.8 * Math.sin(1.93 * t) + 0.35 * (Math.sin(0.31 * t) >= 0 ? 1 : -1);
}

function validationExcitation(t) {
  return 0.95 * Math.sin(0.51 * t + 0.4)
    + 0.55 * Math.sin(2.37 * t + 0.2)
    + 0.28 * (Math.sin(0.22 * t + 0.8) >= 0 ? 1 : -1);
}

function makeEstimator(seed) {
  const sensor = createMeasurementSensor({ C: nominalModel.C, noiseStd: 0.03, seed, bias: 0 });
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
  const initialMeasurement = sensor.read({ x: 0, v: 0 });
  return { sensor, estimator, estimate: estimator.update(initialMeasurement.value) };
}

function identifyW2() {
  const { sensor, estimator } = makeEstimator(20260915);
  let estimate = estimator.getState();
  let truthState = { x: 0, v: 0 };
  const rls = createRecursiveLeastSquaresPlantEstimator({
    dt: cfg.dt,
    initialParameters: { ...nominal },
    initialCovariance: 60,
    forgettingFactor: 0.998,
    bounds: { stiffnessMin: 0.2, stiffnessMax: 4, dampingMin: 0.1, dampingMax: 3, gainMin: 0.2, gainMax: 2 },
  });
  const buffer = [];
  const steps = Math.floor(24 / cfg.dt);

  for (let k = 0; k < steps; k += 1) {
    const t = k * cfg.dt;
    const u = identificationExcitation(t);
    const prev = { x: estimate.x, v: estimate.v };
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    const measurement = sensor.read(nextTruth);
    const nextEstimate = estimator.step(u, measurement.value);
    buffer.push({ state: prev, u });
    if (buffer.length === 2) {
      const avgX = 0.5 * (buffer[0].state.x + buffer[1].state.x);
      const avgV = 0.5 * (buffer[0].state.v + buffer[1].state.v);
      const avgU = 0.5 * (buffer[0].u + buffer[1].u);
      const target = (nextEstimate.v - buffer[0].state.v) / (2 * cfg.dt);
      rls.updateRegression({ regressor: [-avgX, -avgV, avgU], target, context: { aggregationWindow: 2 } });
      buffer.length = 0;
    }
    truthState = nextTruth;
    estimate = nextEstimate;
  }
  return rls.getParameters();
}

function modelConfig(parameters) {
  return { ...cfg, truthPlant: { ...cfg.truthPlant, enabled: false }, plant: { ...parameters } };
}

function rolloutFrom(state, controls, modelCfg) {
  let x = { ...state };
  for (const u of controls) x = stepSecondOrderPlant(x, u, 0, modelCfg);
  return x;
}

function validateCandidate(label, parameters, validationData, horizons) {
  const candidateCfg = modelConfig(parameters);
  const byHorizon = {};
  for (const horizon of horizons) {
    const xErrors = [];
    const vErrors = [];
    for (let k = 0; k + horizon <= validationData.length - 1; k += 1) {
      const start = validationData[k];
      const controls = validationData.slice(k, k + horizon).map((s) => s.u);
      const predicted = rolloutFrom({ x: start.estimateX, v: start.estimateV }, controls, candidateCfg);
      const target = validationData[k + horizon];
      xErrors.push(predicted.x - target.truthX);
      vErrors.push(predicted.v - target.truthV);
    }
    byHorizon[horizon] = { xRmse: rms(xErrors), vRmse: rms(vErrors) };
  }
  return { label, parameters, byHorizon };
}

function collectValidation() {
  const { sensor, estimator } = makeEstimator(20260916);
  let estimate = estimator.getState();
  let truthState = { x: 0, v: 0 };
  const data = [];
  const steps = Math.floor(18 / cfg.dt);
  for (let k = 0; k <= steps; k += 1) {
    const t = k * cfg.dt;
    const u = validationExcitation(t);
    data.push({ t, u, estimateX: estimate.x, estimateV: estimate.v, truthX: truthState.x, truthV: truthState.v });
    if (k === steps) break;
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    const measurement = sensor.read(nextTruth);
    estimate = estimator.step(u, measurement.value);
    truthState = nextTruth;
  }
  return data;
}

const identified = identifyW2();
const candidates = [
  { label: 'nominal', p: nominal },
  { label: 'adapt full W2', p: identified },
  { label: 'adapt c/g only', p: { stiffness: nominal.stiffness, damping: identified.damping, gain: identified.gain } },
  { label: 'adapt k only', p: { stiffness: identified.stiffness, damping: nominal.damping, gain: nominal.gain } },
  { label: 'oracle truth', p: truth },
];
const validationData = collectValidation();
const horizons = [1, 5, 10, 20, 35];
const results = candidates.map(({ label, p }) => validateCandidate(label, p, validationData, horizons));

console.log('Adaptive-model shadow validation on held-out seed/excitation');
console.log('Nominal:', nominal);
console.log('Identified W2:', identified);
console.log('Truth:', truth);
console.table(results.map((r) => ({
  model: r.label,
  k: r.parameters.stiffness.toFixed(4),
  c: r.parameters.damping.toFixed(4),
  g: r.parameters.gain.toFixed(4),
  'xRMSE H1': r.byHorizon[1].xRmse.toFixed(4),
  'vRMSE H1': r.byHorizon[1].vRmse.toFixed(4),
  'xRMSE H10': r.byHorizon[10].xRmse.toFixed(4),
  'vRMSE H10': r.byHorizon[10].vRmse.toFixed(4),
  'xRMSE H35': r.byHorizon[35].xRmse.toFixed(4),
  'vRMSE H35': r.byHorizon[35].vRmse.toFixed(4),
})));

const nominalResult = results.find((r) => r.label === 'nominal');
for (const r of results.filter((item) => item.label !== 'nominal')) {
  console.log(`${r.label}: H35 x ratio=${(r.byHorizon[35].xRmse / nominalResult.byHorizon[35].xRmse).toFixed(3)}, v ratio=${(r.byHorizon[35].vRmse / nominalResult.byHorizon[35].vRmse).toFixed(3)}`);
}
