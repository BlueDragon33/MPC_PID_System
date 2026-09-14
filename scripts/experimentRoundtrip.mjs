import { defaultConfig } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { parseExperimentPayload, serializeExperiment } from '../src/core/experiments/serialization.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const preset = applyExperimentPreset(defaultConfig, 'safety-envelope');
const source = {
  ...preset,
  safety: {
    ...preset.safety,
    previewHorizon: 9,
    positionMargin: 0.03,
    velocityMargin: 0.04,
    outputMargin: 0.02,
  },
};
const encoded = serializeExperiment(source, { name: 'roundtrip', presetId: 'safety-envelope', notes: 'regression' });
const restored = parseExperimentPayload(encoded);

assert(restored.presetId === 'safety-envelope', 'Preset metadata did not survive serialization.');
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

console.log('Experiment serialization roundtrip PASS');
