import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import { applyExperimentPreset } from '../src/core/experiments/presets.js';
import { runAdaptiveModelShadow } from '../src/core/adaptation/runAdaptiveModelShadow.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const preset = applyExperimentPreset(defaultConfig, 'mismatch-observer');
const config = {
  ...preset,
  duration: 12,
  disturbance: { ...preset.disturbance, enabled: true, start: 3.0, duration: 1.2, amplitude: 0.8 },
  mpc: {
    ...preset.mpc,
    horizon: 12,
    qpIterations: Math.max(100, preset.mpc.qpIterations),
    qpProjectionCycles: Math.max(16, preset.mpc.qpProjectionCycles),
    qpTolerance: Math.min(5e-5, preset.mpc.qpTolerance),
  },
};
const result = runSimulation('HYBRID_SAFE', config);
const beforePlant = JSON.stringify(result.config.plant);
const beforeFirstSamples = JSON.stringify(result.samples.slice(0, 5));
const shadow = runAdaptiveModelShadow(result);
const afterPlant = JSON.stringify(result.config.plant);
const afterFirstSamples = JSON.stringify(result.samples.slice(0, 5));

assert(shadow.shadowMode === true, 'Adaptive replay must remain shadow-only.');
assert(shadow.controllerModelUnchanged === true, 'Adaptive replay did not report controller isolation.');
assert(beforePlant === afterPlant, 'Adaptive replay mutated controller plant parameters.');
assert(beforeFirstSamples === afterFirstSamples, 'Adaptive replay mutated simulation samples.');
assert(shadow.events.length > 0, 'Adaptive replay emitted no telemetry events.');
assert(shadow.diagnostics.validationEnabled === true, 'Default adaptive replay must enable holdout validation.');
assert(shadow.diagnostics.minInformationRatio === 1e-4, 'Default adaptive replay PE threshold changed unexpectedly.');
assert(shadow.diagnostics.validationStride === 4, 'Default adaptive replay validation stride changed unexpectedly.');
assert(shadow.diagnostics.validationWindows > 0, 'Adaptive replay produced no holdout windows.');
assert(shadow.diagnostics.rejectedLowInformation > 0, 'Adaptive replay PE gate did not reject any low-information windows.');
assert(shadow.diagnostics.publishCount > 0, 'Adaptive replay never published a validated shadow model.');
assert(result.metrics.maxActualSafetyViolation <= 1e-9, `Baseline simulation safety regressed: ${result.metrics.maxActualSafetyViolation}`);
assert(result.metrics.fallbackCount === 0, `Baseline simulation fallback regressed: ${result.metrics.fallbackCount}`);

console.log('Adaptive simulator shadow replay smoke PASS');
console.table([{
  events: shadow.events.length,
  accepted: shadow.diagnostics.acceptedWindows,
  holdout: shadow.diagnostics.validationWindows,
  peRejected: shadow.diagnostics.rejectedLowInformation,
  validationRejected: shadow.diagnostics.rejectedValidation,
  publishes: shadow.diagnostics.publishCount,
  k: shadow.publishedModel.stiffness.toFixed(4),
  c: shadow.publishedModel.damping.toFixed(4),
  g: shadow.publishedModel.gain.toFixed(4),
}]);
