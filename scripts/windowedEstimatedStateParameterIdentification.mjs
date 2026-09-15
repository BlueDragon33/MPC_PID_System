import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createLinearKalmanFilter } from '../src/core/estimation/linearKalmanFilter.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';
import { createRecursiveLeastSquaresPlantEstimator } from '../src/core/estimation/recursiveLeastSquaresPlantEstimator.js';

const relativeError = (estimate, truth) => Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);

function excitation(t) {
  return 1.2 * Math.sin(0.73 * t)
    + 0.8 * Math.sin(1.93 * t)
    + 0.35 * (Math.sin(0.31 * t) >= 0 ? 1 : -1);
}

const cfg = {
  ...defaultConfig,
  truthPlant: {
    enabled: true,
    stiffnessScale: 1.18,
    dampingScale: 0.78,
    gainScale: 0.90,
  },
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
    bounds: {
      stiffnessMin: 0.2,
      stiffnessMax: 4,
      dampingMin: 0.1,
      dampingMax: 3,
      gainMin: 0.2,
      gainMax: 2,
    },
  });
}

function makeEstimator(kind, sensor) {
  const common = {
    A: model.A,
    B: model.B,
    C: model.C,
    measurementVariance: 0.03 ** 2,
    initialState: kind === 'augmented' ? [0, 0, 0] : [0, 0],
    initialCovariance: kind === 'augmented' ? [0.04, 0.08, 0.2] : [0.04, 0.08],
  };
  const estimator = kind === 'augmented'
    ? createAugmentedDisturbanceKalmanFilter({
        ...common,
        E: model.E,
        processCovariance: [2e-5, 2e-4, 8e-3],
        disturbanceRetention: 0.90,
      })
    : createLinearKalmanFilter({
        ...common,
        processCovariance: [2e-5, 4e-4],
      });
  const initialMeasurement = sensor.read({ x: 0, v: 0 });
  return { estimator, initialEstimate: estimator.update(initialMeasurement.value) };
}

function runWindowed({ kind, windowSize, subtractDisturbance }) {
  const sensor = createMeasurementSensor({ C: model.C, noiseStd: 0.03, seed: 20260915, bias: 0 });
  const { estimator, initialEstimate } = makeEstimator(kind, sensor);
  const rls = makeRls();
  let truthState = { x: 0, v: 0 };
  let estimate = initialEstimate;
  const buffer = [];
  const steps = Math.floor(24 / cfg.dt);

  for (let k = 0; k < steps; k += 1) {
    const t = k * cfg.dt;
    const u = excitation(t);
    const previous = { x: estimate.x, v: estimate.v, d: kind === 'augmented' ? estimate.d : 0 };
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    const measurement = sensor.read(nextTruth);
    const nextEstimate = estimator.step(u, measurement.value);

    buffer.push({ state: previous, u });
    if (buffer.length === windowSize) {
      const avgX = buffer.reduce((sum, item) => sum + item.state.x, 0) / windowSize;
      const avgV = buffer.reduce((sum, item) => sum + item.state.v, 0) / windowSize;
      const avgU = buffer.reduce((sum, item) => sum + item.u, 0) / windowSize;
      const avgD = subtractDisturbance
        ? buffer.reduce((sum, item) => sum + item.state.d, 0) / windowSize
        : 0;
      const target = (nextEstimate.v - buffer[0].state.v) / (windowSize * cfg.dt) - avgD;
      rls.updateRegression({
        regressor: [-avgX, -avgV, avgU],
        target,
        context: {
          aggregationWindow: windowSize,
          disturbanceEstimate: avgD,
          measuredAccelerationWithoutDisturbance: target,
        },
      });
      buffer.length = 0;
    }

    truthState = nextTruth;
    estimate = nextEstimate;
  }

  return {
    parameters: rls.getParameters(),
    diagnostics: rls.getDiagnostics(),
    finalDisturbanceEstimate: kind === 'augmented' ? estimate.d : null,
  };
}

const cases = [];
for (const windowSize of [1, 2, 5, 10, 20]) {
  cases.push({ label: `2-state W${windowSize}`, windowSize, result: runWindowed({ kind: 'linear', windowSize, subtractDisturbance: false }) });
  cases.push({ label: `x-v-d W${windowSize}`, windowSize, result: runWindowed({ kind: 'augmented', windowSize, subtractDisturbance: false }) });
}

const rows = cases.map(({ label, windowSize, result }) => {
  const p = result.parameters;
  const meanError = (
    relativeError(p.stiffness, truth.stiffness)
    + relativeError(p.damping, truth.damping)
    + relativeError(p.gain, truth.gain)
  ) / 3;
  return {
    source: label,
    W: windowSize,
    stiffness: p.stiffness.toFixed(4),
    'k err %': (100 * relativeError(p.stiffness, truth.stiffness)).toFixed(2),
    damping: p.damping.toFixed(4),
    'c err %': (100 * relativeError(p.damping, truth.damping)).toFixed(2),
    gain: p.gain.toFixed(4),
    'g err %': (100 * relativeError(p.gain, truth.gain)).toFixed(2),
    'mean err %': (100 * meanError).toFixed(2),
    residualRMSE: result.diagnostics?.residualRmse?.toFixed(4) ?? 'n/a',
    updates: result.diagnostics?.updates ?? 0,
  };
});

console.log('Windowed estimated-state RLS identification sweep');
console.log(`Truth: stiffness=${truth.stiffness.toFixed(4)}, damping=${truth.damping.toFixed(4)}, gain=${truth.gain.toFixed(4)}`);
console.table(rows);

const ranked = [...rows].sort((a, b) => Number(a['mean err %']) - Number(b['mean err %']));
console.log(`Best mean parameter error: ${ranked[0].source} = ${ranked[0]['mean err %']}%`);
