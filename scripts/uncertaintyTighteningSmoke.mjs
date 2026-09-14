import { defaultConfig } from '../src/core/simulator.js';
import {
  applyCovarianceConstraintTightening,
  computeConstraintTightening,
  covarianceSigmas,
} from '../src/core/estimation/uncertaintyTightening.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function approx(actual, expected, tolerance = 1e-12) {
  assert(Math.abs(actual - expected) <= tolerance, `Expected ${expected}, got ${actual}`);
}

const covariance = [
  [0.04, 0.012],
  [0.012, 0.09],
];
const C = [1, 0];
const sigmas = covarianceSigmas(covariance, C);
approx(sigmas.positionSigma, 0.2);
approx(sigmas.velocitySigma, 0.3);
approx(sigmas.outputSigma, 0.2);

const cfg = {
  ...defaultConfig,
  estimation: {
    ...defaultConfig.estimation,
    enabled: true,
    constraintTighteningEnabled: true,
    constraintSigma: 2,
  },
  mpc: {
    ...defaultConfig.mpc,
    stateConstraintsEnabled: true,
    positionMin: -2,
    positionMax: 2,
    velocityMin: -2,
    velocityMax: 2,
    outputConstraintsEnabled: true,
    outputMin: -1,
    outputMax: 1,
  },
};

const margins = computeConstraintTightening(cfg, covariance, C);
approx(margins.positionMargin, 0.4);
approx(margins.velocityMargin, 0.6);
approx(margins.outputMargin, 0.4);

const tightened = applyCovarianceConstraintTightening(cfg, covariance, C);
assert(tightened.tightening.enabled, 'Tightening should be enabled.');
assert(tightened.tightening.validEnvelope, 'Wide test envelope should remain valid after tightening.');
approx(tightened.config.mpc.positionMin, -1.6);
approx(tightened.config.mpc.positionMax, 1.6);
approx(tightened.config.mpc.velocityMin, -1.4);
approx(tightened.config.mpc.velocityMax, 1.4);
approx(tightened.config.mpc.outputMin, -0.6);
approx(tightened.config.mpc.outputMax, 0.6);

// Original configuration must remain immutable so actual-safety auditing can
// continue against the untightened physical envelope.
approx(cfg.mpc.positionMin, -2);
approx(cfg.mpc.positionMax, 2);
approx(cfg.mpc.outputMin, -1);
approx(cfg.mpc.outputMax, 1);

const impossible = applyCovarianceConstraintTightening({
  ...cfg,
  estimation: { ...cfg.estimation, constraintSigma: 8 },
  mpc: {
    ...cfg.mpc,
    velocityMin: -0.5,
    velocityMax: 0.5,
  },
}, covariance, C);
assert(!impossible.tightening.velocityValid, 'Large covariance should explicitly report an invalid robust velocity envelope.');
assert(!impossible.tightening.validEnvelope, 'Invalid tightened envelope must not be reported as valid.');

const disabled = applyCovarianceConstraintTightening({
  ...cfg,
  estimation: { ...cfg.estimation, constraintTighteningEnabled: false },
}, covariance, C);
assert(!disabled.tightening.enabled, 'Disabled tightening must remain disabled.');
approx(disabled.config.mpc.positionMax, cfg.mpc.positionMax);

console.log('Covariance constraint tightening smoke PASS');
console.log(`Tightening: Δx=${margins.positionMargin.toFixed(3)}, Δv=${margins.velocityMargin.toFixed(3)}, Δy=${margins.outputMargin.toFixed(3)} at ${margins.sigmaMultiplier.toFixed(1)}σ`);
