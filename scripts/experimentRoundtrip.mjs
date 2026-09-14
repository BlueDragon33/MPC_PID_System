import { defaultConfig } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { parseExperimentPayload, serializeExperiment } from '../src/core/experiments/serialization.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
const source = {
  ...preset,
  truthPlant: {
    ...preset.truthPlant,
    enabled: true,
    stiffnessScale: 1.21,
    dampingScale: 0.76,
    gainScale: 0.88,
  },
  safety: {
    ...preset.safety,
    previewHorizon: 9,
    positionMargin: 0.03,
    velocityMargin: 0.04,
    outputMargin: 0.02,
  },
  estimation: {
    ...preset.estimation,
    enabled: true,
    measurementNoiseStd: 0.115,
    measurementBias: 0.007,
    seed: 7319,
    processPositionVariance: 3e-5,
    processVelocityVariance: 4e-4,
    constraintTighteningEnabled: true,
    constraintSigma: 2.5,
    disturbanceStateEnabled: true,
    disturbanceProcessVariance: 0.012,
    initialDisturbanceVariance: 0.65,
  },
};
const encoded = serializeExperiment(source, { name: 'roundtrip', presetId: 'mismatch-observer', notes: 'regression' });
const restored = parseExperimentPayload(encoded);

assert(restored.presetId === 'mismatch-observer', 'Preset metadata did not survive serialization.');
assert(restored.config.mpc.stateConstraintsEnabled === true, 'State constraint flag did not survive serialization.');
assert(restored.config.mpc.outputConstraintsEnabled === true, 'Output constraint flag did not survive serialization.');
assert(restored.config.mpc.velocityMax === source.mpc.velocityMax, 'Velocity envelope changed during roundtrip.');
assert(restored.config.mpc.outputMax === source.mpc.outputMax, 'Output envelope changed during roundtrip.');
assert(restored.config.mpc.deltaUMax === source.mpc.deltaUMax, 'Slew-rate limit changed during roundtrip.');
assert(restored.config.disturbance.amplitude === source.disturbance.amplitude, 'Disturbance configuration changed during roundtrip.');
assert(restored.config.safety.previewHorizon === source.safety.previewHorizon, 'Governor preview horizon changed during roundtrip.');
assert(restored.config.safety.positionMargin === source.safety.positionMargin, 'Governor position margin changed during roundtrip.');
assert(restored.config.safety.velocityMargin === source.safety.velocityMargin, 'Governor velocity margin changed during roundtrip.');
assert(restored.config.safety.outputMargin === source.safety.outputMargin, 'Governor output margin changed during roundtrip.');
assert(restored.config.truthPlant.enabled === true, 'Truth-plant mismatch flag did not survive serialization.');
assert(restored.config.truthPlant.stiffnessScale === source.truthPlant.stiffnessScale, 'Truth stiffness scale changed during roundtrip.');
assert(restored.config.truthPlant.dampingScale === source.truthPlant.dampingScale, 'Truth damping scale changed during roundtrip.');
assert(restored.config.truthPlant.gainScale === source.truthPlant.gainScale, 'Truth gain scale changed during roundtrip.');
assert(restored.config.estimation.enabled === true, 'Estimation enabled flag did not survive serialization.');
assert(restored.config.estimation.measurementNoiseStd === source.estimation.measurementNoiseStd, 'Measurement noise changed during roundtrip.');
assert(restored.config.estimation.measurementBias === source.estimation.measurementBias, 'Measurement bias changed during roundtrip.');
assert(restored.config.estimation.seed === source.estimation.seed, 'Estimation seed changed during roundtrip.');
assert(restored.config.estimation.processPositionVariance === source.estimation.processPositionVariance, 'Position process variance changed during roundtrip.');
assert(restored.config.estimation.processVelocityVariance === source.estimation.processVelocityVariance, 'Velocity process variance changed during roundtrip.');
assert(restored.config.estimation.constraintTighteningEnabled === true, 'Constraint-tightening flag did not survive serialization.');
assert(restored.config.estimation.constraintSigma === source.estimation.constraintSigma, 'Constraint sigma changed during roundtrip.');
assert(restored.config.estimation.disturbanceStateEnabled === true, 'Disturbance-state flag did not survive serialization.');
assert(restored.config.estimation.disturbanceProcessVariance === source.estimation.disturbanceProcessVariance, 'Disturbance process variance changed during roundtrip.');
assert(restored.config.estimation.initialDisturbanceVariance === source.estimation.initialDisturbanceVariance, 'Initial disturbance variance changed during roundtrip.');

console.log('Experiment serialization roundtrip PASS');
