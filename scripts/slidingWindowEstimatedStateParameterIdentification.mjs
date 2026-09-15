import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';
import { createRecursiveLeastSquaresPlantEstimator } from '../src/core/estimation/recursiveLeastSquaresPlantEstimator.js';

const relativeError = (estimate, truth) => Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);
function excitation(t) {
  return 1.2 * Math.sin(0.73 * t) + 0.8 * Math.sin(1.93 * t) + 0.35 * (Math.sin(0.31 * t) >= 0 ? 1 : -1);
}

const cfg = {
  ...defaultConfig,
  truthPlant: { enabled: true, stiffnessScale: 1.18, dampingScale: 0.78, gainScale: 0.90 },
};
const model = createSecondOrderModel(cfg);
const truthCfg = createTruthPlantConfig(cfg);
const truth = truthCfg.plant;

function makeRls() {
  return createRecursiveLeastSquaresPlantEstimator({
    dt: cfg.dt,
    initialParameters: { ...cfg.plant },
    initialCovariance: 60,
    forgettingFactor: 0.998,
    bounds: { stiffnessMin: 0.2, stiffnessMax: 4, dampingMin: 0.1, dampingMax: 3, gainMin: 0.2, gainMax: 2 },
  });
}

function run(windowSize, trapezoidal = false) {
  const sensor = createMeasurementSensor({ C: model.C, noiseStd: 0.03, seed: 20260915, bias: 0 });
  const estimator = createAugmentedDisturbanceKalmanFilter({
    A: model.A, B: model.B, E: model.E, C: model.C,
    processCovariance: [2e-5, 2e-4, 8e-3],
    measurementVariance: 0.03 ** 2,
    initialState: [0, 0, 0],
    initialCovariance: [0.04, 0.08, 0.2],
    disturbanceRetention: 0.90,
  });
  const initialMeasurement = sensor.read({ x: 0, v: 0 });
  let estimate = estimator.update(initialMeasurement.value);
  let truthState = { x: 0, v: 0 };
  const rls = makeRls();
  const transitions = [];
  const steps = Math.floor(24 / cfg.dt);

  for (let k = 0; k < steps; k += 1) {
    const t = k * cfg.dt;
    const u = excitation(t);
    const previous = { x: estimate.x, v: estimate.v };
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    const measurement = sensor.read(nextTruth);
    const nextEstimate = estimator.step(u, measurement.value);
    transitions.push({ x: previous.x, v: previous.v, u, nextV: nextEstimate.v });
    if (transitions.length > windowSize) transitions.shift();

    if (transitions.length === windowSize) {
      let avgX;
      let avgV;
      let avgU;
      if (trapezoidal && windowSize > 1) {
        const weights = transitions.map((_, i) => (i === 0 || i === windowSize - 1 ? 0.5 : 1));
        const denom = weights.reduce((a, b) => a + b, 0);
        avgX = transitions.reduce((sum, item, i) => sum + weights[i] * item.x, 0) / denom;
        avgV = transitions.reduce((sum, item, i) => sum + weights[i] * item.v, 0) / denom;
        avgU = transitions.reduce((sum, item, i) => sum + weights[i] * item.u, 0) / denom;
      } else {
        avgX = transitions.reduce((sum, item) => sum + item.x, 0) / windowSize;
        avgV = transitions.reduce((sum, item) => sum + item.v, 0) / windowSize;
        avgU = transitions.reduce((sum, item) => sum + item.u, 0) / windowSize;
      }
      const target = (nextEstimate.v - transitions[0].v) / (windowSize * cfg.dt);
      rls.updateRegression({
        regressor: [-avgX, -avgV, avgU],
        target,
        context: { aggregationWindow: windowSize, overlapping: true, trapezoidal },
      });
    }

    truthState = nextTruth;
    estimate = nextEstimate;
  }

  const p = rls.getParameters();
  const errors = [relativeError(p.stiffness, truth.stiffness), relativeError(p.damping, truth.damping), relativeError(p.gain, truth.gain)];
  return { parameters: p, diagnostics: rls.getDiagnostics(), meanError: errors.reduce((a, b) => a + b, 0) / errors.length };
}

const cases = [];
for (const W of [2, 3, 4, 5]) {
  cases.push({ label: `sliding W${W}`, W, result: run(W, false) });
  cases.push({ label: `trap W${W}`, W, result: run(W, true) });
}

const rows = cases.map(({ label, W, result }) => ({
  source: label,
  W,
  stiffness: result.parameters.stiffness.toFixed(4),
  'k err %': (100 * relativeError(result.parameters.stiffness, truth.stiffness)).toFixed(2),
  damping: result.parameters.damping.toFixed(4),
  'c err %': (100 * relativeError(result.parameters.damping, truth.damping)).toFixed(2),
  gain: result.parameters.gain.toFixed(4),
  'g err %': (100 * relativeError(result.parameters.gain, truth.gain)).toFixed(2),
  'mean err %': (100 * result.meanError).toFixed(2),
  residualRMSE: result.diagnostics?.residualRmse?.toFixed(4) ?? 'n/a',
  updates: result.diagnostics?.updates ?? 0,
}));

console.log('Sliding-window estimated-state RLS identification sweep');
console.log(`Truth: stiffness=${truth.stiffness.toFixed(4)}, damping=${truth.damping.toFixed(4)}, gain=${truth.gain.toFixed(4)}`);
console.table(rows);
const ranked = [...rows].sort((a, b) => Number(a['mean err %']) - Number(b['mean err %']));
console.log(`Best sliding-window mean parameter error: ${ranked[0].source} = ${ranked[0]['mean err %']}%`);
