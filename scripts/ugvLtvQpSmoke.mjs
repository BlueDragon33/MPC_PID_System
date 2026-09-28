import assert from 'node:assert/strict';

import { createUgvInitialState } from '../src/core/models/kinematicBicycle.js';
import { buildUgvLtvQp, solveUgvLtvMpc } from '../src/core/mpc/ugvLtvMpc.js';
import { mergeUgvConfig } from '../src/core/orchestration/ugvSimulationConfig.js';
import { inequalityViolation } from '../src/core/mpc/constraints.js';

const cfg = mergeUgvConfig();
const state = createUgvInitialState({ y: 0.6, v: 0 });
const previousCommand = { acceleration: 0, steering: 0 };
const qp = buildUgvLtvQp(state, previousCommand, cfg);
const dimension = cfg.ugvLtvMpc.horizon * 2;

assert.equal(qp.H.length, dimension);
assert.equal(qp.H[0].length, dimension);
assert.equal(qp.f.length, dimension);
assert.equal(qp.inequalities.dimension, dimension);
assert(qp.inequalities.rows.length > dimension, 'UGV QP must include hard input/rate/state constraints');
assert(qp.H.flat().every(Number.isFinite));
assert(qp.f.every(Number.isFinite));

const result = solveUgvLtvMpc(state, previousCommand, cfg);
assert(['solved', 'max-iterations'].includes(result.status), `UGV LTV QP status=${result.status}`);
assert.equal(result.fallbackUsed, false);
assert(Number.isFinite(result.command.acceleration));
assert(Number.isFinite(result.command.steering));
const feasibility = inequalityViolation(qp.inequalities, result.sequence);
assert(
  feasibility.maxViolation <= cfg.ugvLtvMpc.qpFeasibilityTolerance * 10,
  `UGV LTV QP violation=${feasibility.maxViolation}`,
);
assert(result.command.acceleration <= cfg.ugv.accelerationMax + 1e-9);
assert(result.command.acceleration >= cfg.ugv.accelerationMin - 1e-9);
assert(result.command.steering <= cfg.ugv.steeringMax + 1e-9);
assert(result.command.steering >= cfg.ugv.steeringMin - 1e-9);

console.log('UGV LTV QP smoke PASS', {
  dimension,
  inequalities: qp.inequalities.rows.length,
  status: result.status,
  iterations: result.diagnostics.iterations,
  feasibility: result.diagnostics.feasibilityViolation,
  command: result.command,
});
