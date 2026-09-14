import { defaultConfig } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { parseExperimentPayload, serializeExperiment } from '../src/core/experiments/serialization.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const source = applyExperimentPreset(defaultConfig, 'safety-envelope');
const encoded = serializeExperiment(source, { name: 'roundtrip', presetId: 'safety-envelope', notes: 'regression' });
const restored = parseExperimentPayload(encoded);

assert(restored.presetId === 'safety-envelope', 'Preset metadata did not survive serialization.');
assert(restored.config.mpc.stateConstraintsEnabled === true, 'State constraint flag did not survive serialization.');
assert(restored.config.mpc.outputConstraintsEnabled === true, 'Output constraint flag did not survive serialization.');
assert(restored.config.mpc.velocityMax === source.mpc.velocityMax, 'Velocity envelope changed during roundtrip.');
assert(restored.config.mpc.outputMax === source.mpc.outputMax, 'Output envelope changed during roundtrip.');
assert(restored.config.mpc.deltaUMax === source.mpc.deltaUMax, 'Slew-rate limit changed during roundtrip.');
assert(restored.config.disturbance.amplitude === source.disturbance.amplitude, 'Disturbance configuration changed during roundtrip.');

console.log('Experiment serialization roundtrip PASS');
