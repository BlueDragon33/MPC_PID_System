import { compareControllers, defaultConfig, getStateSpaceModel } from '../src/core/simulator.js';
import { buildCondensedQP, evaluateCondensedQP, qpDiagnostics } from '../src/core/mpc/condensedQP.js';
import { evaluateMPCSequenceCost } from '../src/core/solvers/projectedGradientMPC.js';

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

const model = getStateSpaceModel(defaultConfig);
const qpState = { x: 0.2, v: -0.1 };
const qpPreviousU = 0.3;
const qp = buildCondensedQP({
  A: model.A,
  B: model.B,
  state: qpState,
  target: defaultConfig.setpoint,
  previousU: qpPreviousU,
  cfg: defaultConfig,
});
const qpInfo = qpDiagnostics(qp);
assert(qp.H.length === defaultConfig.mpc.horizon, 'QP Hessian dimension must match control horizon');
assert(qp.H.every((row) => row.length === defaultConfig.mpc.horizon), 'QP Hessian must be square');
assert(qp.f.length === defaultConfig.mpc.horizon, 'QP linear term dimension must match horizon');
assert(qp.lower.length === defaultConfig.mpc.horizon && qp.upper.length === defaultConfig.mpc.horizon, 'QP bound dimensions must match horizon');
assert(qpInfo.finite, 'QP formulation contains non-finite values');
assert(qpInfo.maxSymmetryError < 1e-9, `QP Hessian is not symmetric: ${qpInfo.maxSymmetryError}`);
assert(qpInfo.minDiagonal > 0, 'QP Hessian diagonal must be positive for the default problem');

const U0 = new Array(defaultConfig.mpc.horizon).fill(0);
const U1 = Array.from({ length: defaultConfig.mpc.horizon }, (_, i) => 0.15 + 0.22 * Math.sin(i * 0.31));
const direct0 = evaluateMPCSequenceCost(qpState, U0, defaultConfig.setpoint, qpPreviousU, defaultConfig);
const direct1 = evaluateMPCSequenceCost(qpState, U1, defaultConfig.setpoint, qpPreviousU, defaultConfig);
const condensed0 = evaluateCondensedQP(qp, U0);
const condensed1 = evaluateCondensedQP(qp, U1);
const objectiveDeltaError = Math.abs((direct1 - direct0) - (condensed1 - condensed0));
assert(objectiveDeltaError < 1e-8, `Condensed QP objective does not match rollout cost: ${objectiveDeltaError}`);

console.log('MPC_PID_System smoke test PASS');
for (const result of results) {
  console.log(`${result.mode}: IAE=${result.metrics.iae.toFixed(4)}, solves=${result.metrics.solveCount}, reduction=${result.metrics.computeReduction.toFixed(1)}%`);
}
console.log(`QP: n=${qpInfo.dimension}, symmetryError=${qpInfo.maxSymmetryError.toExponential(2)}, minDiagonal=${qpInfo.minDiagonal.toFixed(6)}, objectiveDeltaError=${objectiveDeltaError.toExponential(2)}`);
