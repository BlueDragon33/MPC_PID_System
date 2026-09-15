import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel } from '../src/core/models/secondOrderPlant.js';
import { buildCondensedQP } from '../src/core/mpc/condensedQP.js';
import { rolloutMPCSequence } from '../src/core/mpc/rollout.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function maxAbsDiff(a, b) {
  let max = 0;
  for (let i = 0; i < a.length; i += 1) max = Math.max(max, Math.abs(a[i] - b[i]));
  return max;
}

function flattenMatrix(A) {
  return A.flat();
}

const horizon = 9;
const state = { x: 0.32, v: -0.18 };
const previousU = 0.14;
const dHat = 0.87;
const rho = 0.9;
const config = {
  ...defaultConfig,
  mpc: {
    ...defaultConfig.mpc,
    horizon,
    stateConstraintsEnabled: false,
    outputConstraintsEnabled: false,
  },
  estimation: {
    ...defaultConfig.estimation,
    mpcDisturbanceCompensationEnabled: true,
    disturbanceRetention: rho,
  },
  runtime: {
    disturbanceEstimate: dHat,
    disturbanceRetention: rho,
  },
};

const model = createSecondOrderModel(config);
const compensated = buildCondensedQP({
  A: model.A,
  B: model.B,
  E: model.E,
  C: model.C,
  state,
  target: 1,
  previousU,
  cfg: config,
});
const zeroSequence = new Array(horizon).fill(0);
const rollout = rolloutMPCSequence(state, zeroSequence, 1, previousU, config);
const rolloutStates = rollout.xs.slice(1).flatMap((sample) => [sample.x, sample.v]);

const nominalConfig = {
  ...config,
  estimation: {
    ...config.estimation,
    mpcDisturbanceCompensationEnabled: false,
  },
};
const nominal = buildCondensedQP({
  A: model.A,
  B: model.B,
  E: model.E,
  C: model.C,
  state,
  target: 1,
  previousU,
  cfg: nominalConfig,
});

const freePredictionError = maxAbsDiff(compensated.freePrediction, rolloutStates);
const hessianDelta = maxAbsDiff(flattenMatrix(compensated.H), flattenMatrix(nominal.H));
const disturbanceOffsetMagnitude = Math.max(...compensated.disturbanceOffset.map(Math.abs));
const linearTermDelta = maxAbsDiff(compensated.f, nominal.f);

assert(compensated.disturbanceCompensationEnabled === true, 'Affine disturbance compensation flag was not propagated into the condensed QP.');
assert(freePredictionError < 1e-12, `Condensed affine free prediction disagrees with rollout: ${freePredictionError}`);
assert(hessianDelta < 1e-12, `Affine disturbance should not change the QP Hessian: ${hessianDelta}`);
assert(disturbanceOffsetMagnitude > 1e-6, 'Disturbance offset unexpectedly vanished.');
assert(linearTermDelta > 1e-6, 'Affine disturbance did not shift the QP linear term.');

console.log('Affine disturbance condensed-QP smoke PASS');
console.log(`AffineQP: freePredictionError=${freePredictionError.toExponential(2)}, Hdelta=${hessianDelta.toExponential(2)}, maxOffset=${disturbanceOffsetMagnitude.toFixed(6)}, fDelta=${linearTermDelta.toFixed(6)}`);
