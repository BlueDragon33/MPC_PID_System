import { compareControllers, defaultConfig, getStateSpaceModel } from '../src/core/simulator.js';
import { buildCondensedQP, evaluateCondensedQP, qpDiagnostics } from '../src/core/mpc/condensedQP.js';
import { inequalityViolation } from '../src/core/mpc/constraints.js';
import { evaluateMPCSequenceCost } from '../src/core/solvers/projectedGradientMPC.js';
import { solveBoxQPMPC } from '../src/core/solvers/boxQPMPC.js';
import { solveConstrainedQPMPC } from '../src/core/solvers/constrainedQPMPC.js';

const results = compareControllers(defaultConfig);
const byMode = Object.fromEntries(results.map((r) => [r.mode, r]));

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function assertRateBounds(sequence, previousU, minDelta, maxDelta, tolerance = 1e-7) {
  let prev = previousU;
  for (let i = 0; i < sequence.length; i += 1) {
    const delta = sequence[i] - prev;
    assert(delta >= minDelta - tolerance, `rate lower bound violated at ${i}: ${delta}`);
    assert(delta <= maxDelta + tolerance, `rate upper bound violated at ${i}: ${delta}`);
    prev = sequence[i];
  }
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
assert(Number.isFinite(byMode.HYBRID.metrics.avgIterations), 'Hybrid solver iteration metric must be finite');
assert(byMode.HYBRID.metrics.maxFeasibilityViolation <= 1e-6, 'Hybrid QP must remain feasible under hard inequalities');
assert(byMode.HYBRID.metrics.infeasibleCount === 0, 'Default hybrid experiment must not become infeasible');
assert(byMode.HYBRID.metrics.numericalFailureCount === 0, 'Default hybrid experiment must not have numerical failures');

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
assert(qpInfo.rateConstraintsEnabled, 'Default QP must include hard delta-u constraints');
assert(qpInfo.inequalities === 4 * defaultConfig.mpc.horizon, 'Expected input and rate upper/lower inequalities at every horizon stage');
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

const boxConfig = {
  ...defaultConfig,
  mpc: {
    ...defaultConfig.mpc,
    horizon: 14,
    qpIterations: 300,
    qpTolerance: 1e-6,
    deltaUMin: Number.NEGATIVE_INFINITY,
    deltaUMax: Number.POSITIVE_INFINITY,
  },
};
const boxWarmStart = new Array(boxConfig.mpc.horizon).fill(qpPreviousU);
const boxSolution = solveBoxQPMPC(qpState, boxConfig.setpoint, qpPreviousU, boxConfig, boxWarmStart);
assert(boxSolution.solver === 'box-qp', 'Expected the box QP baseline backend');
assert(boxSolution.diagnostics.finite, 'Box QP diagnostics must be finite');
assert(boxSolution.diagnostics.feasibilityViolation <= 1e-10, 'Box QP solution violates input bounds');
assert(boxSolution.sequence.every((u) => u >= boxConfig.mpc.uMin - 1e-10 && u <= boxConfig.mpc.uMax + 1e-10), 'Box QP control sequence violates bounds');

const constrainedConfig = {
  ...defaultConfig,
  mpc: {
    ...defaultConfig.mpc,
    horizon: 14,
    qpIterations: 180,
    qpTolerance: 1e-5,
    qpProjectionCycles: 10,
    deltaUMin: -0.25,
    deltaUMax: 0.25,
  },
};
const constrainedWarmStart = new Array(constrainedConfig.mpc.horizon).fill(qpPreviousU);
const constrainedSolution = solveConstrainedQPMPC(qpState, constrainedConfig.setpoint, qpPreviousU, constrainedConfig, constrainedWarmStart);
assert(constrainedSolution.solver === 'constrained-qp', 'Expected constrained QP backend');
assert(!constrainedSolution.fallbackUsed, `Constrained QP unexpectedly used fallback: ${constrainedSolution.fallbackReason}`);
assert(constrainedSolution.diagnostics.finite, 'Constrained QP diagnostics must be finite');
assert(constrainedSolution.diagnostics.feasibilityViolation <= 1e-6, 'Constrained QP solution violates AU<=b');
assertRateBounds(constrainedSolution.sequence, qpPreviousU, constrainedConfig.mpc.deltaUMin, constrainedConfig.mpc.deltaUMax);
const constrainedQP = buildCondensedQP({ A: model.A, B: model.B, state: qpState, target: constrainedConfig.setpoint, previousU: qpPreviousU, cfg: constrainedConfig });
assert(inequalityViolation(constrainedQP.inequalities, constrainedSolution.sequence).maxViolation <= 1e-6, 'Constrained QP sequence fails independent inequality check');

const impossibleConfig = {
  ...constrainedConfig,
  mpc: {
    ...constrainedConfig.mpc,
    uMin: -1,
    uMax: 1,
    deltaUMin: 0.2,
    deltaUMax: 0.4,
  },
};
const impossibleSolution = solveConstrainedQPMPC(qpState, impossibleConfig.setpoint, 1, impossibleConfig, null);
assert(impossibleSolution.status === 'infeasible', 'Conflicting first-move constraints must report infeasible');
assert(impossibleSolution.fallbackUsed, 'Infeasible QP must activate fallback semantics');

console.log('MPC_PID_System smoke test PASS');
for (const result of results) {
  const convergence = result.metrics.convergenceRate == null ? 'n/a' : `${result.metrics.convergenceRate.toFixed(1)}%`;
  console.log(`${result.mode}: IAE=${result.metrics.iae.toFixed(4)}, solves=${result.metrics.solveCount}, reduction=${result.metrics.computeReduction.toFixed(1)}%, convergence=${convergence}, fallback=${result.metrics.fallbackCount}`);
}
console.log(`QP: n=${qpInfo.dimension}, inequalities=${qpInfo.inequalities}, symmetryError=${qpInfo.maxSymmetryError.toExponential(2)}, objectiveDeltaError=${objectiveDeltaError.toExponential(2)}`);
console.log(`ConstrainedQP: status=${constrainedSolution.status}, residual=${constrainedSolution.diagnostics.projectedGradientResidual.toExponential(2)}, feasibility=${constrainedSolution.diagnostics.feasibilityViolation.toExponential(2)}, projections=${constrainedSolution.diagnostics.projectionCycles}`);
