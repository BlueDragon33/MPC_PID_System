import { compareControllers, defaultConfig } from '../src/core/simulator.js';

const results = compareControllers(defaultConfig);
const byMode = Object.fromEntries(results.map((r) => [r.mode, r]));

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

for (const result of results) {
  assert(result.samples.length > 10, `${result.mode}: simulation produced too few samples`);
  assert(result.samples.every((p) => Number.isFinite(p.x) && Number.isFinite(p.v) && Number.isFinite(p.u)), `${result.mode}: non-finite state/control detected`);
  assert(Number.isFinite(result.metrics.iae), `${result.mode}: invalid IAE`);
  assert(Number.isFinite(result.metrics.controlEffort), `${result.mode}: invalid control effort`);
}

assert(byMode.MPC.metrics.solveCount === byMode.MPC.samples.length, 'Periodic MPC must solve every sample');
assert(byMode.HYBRID.metrics.solveCount > 0, 'Hybrid controller must invoke MPC');
assert(byMode.HYBRID.metrics.solveCount < byMode.MPC.metrics.solveCount, 'Event-triggered hybrid must solve less often than periodic MPC');
assert(byMode.HYBRID.metrics.computeReduction > 50, 'Default hybrid configuration should avoid at least 50% of periodic MPC solves');

const disturbanceSamples = byMode.HYBRID.samples.filter((p) => Math.abs(p.disturbance) > 1e-9);
assert(disturbanceSamples.length > 0, 'Default experiment must contain disturbance injection');

console.log('MPC_PID_System smoke test PASS');
for (const result of results) {
  console.log(`${result.mode}: IAE=${result.metrics.iae.toFixed(4)}, solves=${result.metrics.solveCount}, reduction=${result.metrics.computeReduction.toFixed(1)}%`);
}
