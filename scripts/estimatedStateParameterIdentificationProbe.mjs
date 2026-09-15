import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createLinearKalmanFilter } from '../src/core/estimation/linearKalmanFilter.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';
import { createRecursiveLeastSquaresPlantEstimator } from '../src/core/estimation/recursiveLeastSquaresPlantEstimator.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function relativeError(estimate, truth) {
  return Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);
}

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

function makeStateEstimator(kind, sensor) {
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

function runTruthStateBaseline() {
  const rls = makeRls();
  let truthState = { x: 0, v: 0 };
  const steps = Math.floor(18 / cfg.dt);
  for (let k = 0; k < steps; k += 1) {
    const t = k * cfg.dt;
    const u = excitation(t);
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    rls.update({ previousState: truthState, nextState: nextTruth, u, disturbanceEstimate: 0 });
    truthState = nextTruth;
  }
  return { parameters: rls.getParameters(), diagnostics: rls.getDiagnostics() };
}

function runEstimatedCase({ kind, subtractDisturbance }) {
  const sensor = createMeasurementSensor({ C: model.C, noiseStd: 0.03, seed: 20260915, bias: 0 });
  const { estimator, initialEstimate } = makeStateEstimator(kind, sensor);
  const rls = makeRls();
  let truthState = { x: 0, v: 0 };
  let estimate = initialEstimate;
  const steps = Math.floor(18 / cfg.dt);

  for (let k = 0; k < steps; k += 1) {
    const t = k * cfg.dt;
    const u = excitation(t);
    const previousEstimateState = { x: estimate.x, v: estimate.v };
    const transitionDisturbanceEstimate = kind === 'augmented' && subtractDisturbance ? estimate.d : 0;
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    const measurement = sensor.read(nextTruth);
    const nextEstimate = estimator.step(u, measurement.value);

    rls.update({
      previousState: previousEstimateState,
      nextState: { x: nextEstimate.x, v: nextEstimate.v },
      u,
      disturbanceEstimate: transitionDisturbanceEstimate,
    });

    truthState = nextTruth;
    estimate = nextEstimate;
  }

  return {
    parameters: rls.getParameters(),
    diagnostics: rls.getDiagnostics(),
    finalDisturbanceEstimate: kind === 'augmented' ? estimate.d : null,
  };
}

const cases = [
  { label: 'truth-state', result: runTruthStateBaseline() },
  { label: '2-state KF · raw residual', result: runEstimatedCase({ kind: 'linear', subtractDisturbance: false }) },
  { label: 'x-v-d KF · raw residual', result: runEstimatedCase({ kind: 'augmented', subtractDisturbance: false }) },
  { label: 'x-v-d KF · subtract d-hat', result: runEstimatedCase({ kind: 'augmented', subtractDisturbance: true }) },
];

const rows = cases.map(({ label, result }) => {
  const p = result.parameters;
  return {
    source: label,
    stiffness: p.stiffness.toFixed(4),
    'k err %': (100 * relativeError(p.stiffness, truth.stiffness)).toFixed(2),
    damping: p.damping.toFixed(4),
    'c err %': (100 * relativeError(p.damping, truth.damping)).toFixed(2),
    gain: p.gain.toFixed(4),
    'g err %': (100 * relativeError(p.gain, truth.gain)).toFixed(2),
    residualRMSE: result.diagnostics?.residualRmse?.toFixed(4) ?? 'n/a',
    covarianceTrace: result.diagnostics?.covarianceTrace?.toFixed(4) ?? 'n/a',
    finalDhat: result.finalDisturbanceEstimate == null ? '—' : result.finalDisturbanceEstimate.toFixed(4),
  };
});

console.log('Estimated-state RLS parameter-identification probe');
console.log(`Truth: stiffness=${truth.stiffness.toFixed(4)}, damping=${truth.damping.toFixed(4)}, gain=${truth.gain.toFixed(4)}`);
console.table(rows);

for (const { label, result } of cases) {
  const p = result.parameters;
  assert(Number.isFinite(p.stiffness) && Number.isFinite(p.damping) && Number.isFinite(p.gain), `${label}: non-finite RLS parameters.`);
  assert(p.stiffness >= 0.2 && p.stiffness <= 4, `${label}: stiffness escaped physical bounds.`);
  assert(p.damping >= 0.1 && p.damping <= 3, `${label}: damping escaped physical bounds.`);
  assert(p.gain >= 0.2 && p.gain <= 2, `${label}: gain escaped physical bounds.`);
  assert((result.diagnostics?.updates ?? 0) > 700, `${label}: insufficient RLS updates.`);
}

console.log('Estimated-state RLS parameter-identification probe PASS');
