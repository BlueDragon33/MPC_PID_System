import { defaultConfig } from '../src/core/simulator.js';
import { createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createRecursiveLeastSquaresPlantEstimator } from '../src/core/estimation/recursiveLeastSquaresPlantEstimator.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function relativeError(estimate, truth) {
  return Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);
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
const truthCfg = createTruthPlantConfig(cfg);
const truth = truthCfg.plant;
const estimator = createRecursiveLeastSquaresPlantEstimator({
  dt: cfg.dt,
  initialParameters: { ...cfg.plant },
  initialCovariance: 100,
  forgettingFactor: 0.997,
  bounds: {
    stiffnessMin: 0.2,
    stiffnessMax: 4,
    dampingMin: 0.1,
    dampingMax: 3,
    gainMin: 0.2,
    gainMax: 2,
  },
});

let state = { x: 0, v: 0 };
const duration = 16;
const steps = Math.floor(duration / cfg.dt);
for (let k = 0; k < steps; k += 1) {
  const t = k * cfg.dt;
  const u = 1.2 * Math.sin(0.73 * t)
    + 0.8 * Math.sin(1.93 * t)
    + 0.35 * (Math.sin(0.31 * t) >= 0 ? 1 : -1);
  const next = stepSecondOrderPlant(state, u, 0, truthCfg);
  estimator.update({
    previousState: state,
    nextState: next,
    u,
    disturbanceEstimate: 0,
  });
  state = next;
}

const estimate = estimator.getParameters();
const diagnostics = estimator.getDiagnostics();
const errors = {
  stiffness: relativeError(estimate.stiffness, truth.stiffness),
  damping: relativeError(estimate.damping, truth.damping),
  gain: relativeError(estimate.gain, truth.gain),
};

assert(errors.stiffness < 0.005, `RLS stiffness relative error too high: ${errors.stiffness}`);
assert(errors.damping < 0.005, `RLS damping relative error too high: ${errors.damping}`);
assert(errors.gain < 0.005, `RLS gain relative error too high: ${errors.gain}`);
assert((diagnostics?.updates ?? 0) > 500, 'RLS did not receive enough informative updates.');
assert(Number.isFinite(diagnostics?.residualRmse), 'RLS residual RMSE is not finite.');
assert((diagnostics?.covarianceTrace ?? Infinity) < 0.1, `RLS covariance did not contract sufficiently: ${diagnostics?.covarianceTrace}`);

console.log('RLS plant-parameter identification smoke PASS');
console.table([{
  parameter: 'stiffness', truth: truth.stiffness.toFixed(6), estimate: estimate.stiffness.toFixed(6), 'relative error %': (100 * errors.stiffness).toFixed(4),
}, {
  parameter: 'damping', truth: truth.damping.toFixed(6), estimate: estimate.damping.toFixed(6), 'relative error %': (100 * errors.damping).toFixed(4),
}, {
  parameter: 'gain', truth: truth.gain.toFixed(6), estimate: estimate.gain.toFixed(6), 'relative error %': (100 * errors.gain).toFixed(4),
}]);
console.log(`RLS: updates=${diagnostics.updates}, residualRMSE=${diagnostics.residualRmse.toExponential(3)}, covarianceTrace=${diagnostics.covarianceTrace.toExponential(3)}`);
