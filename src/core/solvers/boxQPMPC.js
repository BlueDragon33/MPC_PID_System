import { createSecondOrderModel } from '../models/secondOrderPlant.js';
import { buildCondensedQP, evaluateCondensedQP } from '../mpc/condensedQP.js';
import { rolloutMPCSequence } from '../mpc/rollout.js';

const now = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());
const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

function matVec(A, x) {
  return A.map((row) => row.reduce((sum, value, j) => sum + value * x[j], 0));
}

function gradient(qp, x) {
  const Hx = matVec(qp.H, x);
  return Hx.map((value, i) => value + qp.f[i]);
}

function project(qp, x) {
  return x.map((value, i) => clamp(value, qp.lower[i], qp.upper[i]));
}

function maxAbs(values) {
  let result = 0;
  for (const value of values) result = Math.max(result, Math.abs(value));
  return result;
}

function maxRowAbsSum(H) {
  let result = 0;
  for (const row of H) {
    result = Math.max(result, row.reduce((sum, value) => sum + Math.abs(value), 0));
  }
  return Math.max(result, 1e-9);
}

function feasibilityViolation(qp, x) {
  let violation = 0;
  for (let i = 0; i < x.length; i += 1) {
    violation = Math.max(violation, qp.lower[i] - x[i], x[i] - qp.upper[i], 0);
  }
  return violation;
}

function kktResidual(qp, x, boundTolerance = 1e-8) {
  const g = gradient(qp, x);
  let residual = 0;

  for (let i = 0; i < x.length; i += 1) {
    const atLower = x[i] <= qp.lower[i] + boundTolerance;
    const atUpper = x[i] >= qp.upper[i] - boundTolerance;
    let violation;

    if (atLower && atUpper) violation = 0;
    else if (atLower) violation = Math.max(0, -g[i]);
    else if (atUpper) violation = Math.max(0, g[i]);
    else violation = Math.abs(g[i]);

    residual = Math.max(residual, violation);
  }

  return residual;
}

function activeConstraintStats(qp, x, tolerance = 1e-6) {
  let lower = 0;
  let upper = 0;
  for (let i = 0; i < x.length; i += 1) {
    if (Math.abs(x[i] - qp.lower[i]) <= tolerance) lower += 1;
    else if (Math.abs(x[i] - qp.upper[i]) <= tolerance) upper += 1;
  }
  return {
    lower,
    upper,
    total: lower + upper,
    ratio: x.length ? (lower + upper) / x.length : 0,
  };
}

function shiftedWarmStart(warmStart, horizon, previousU) {
  if (!warmStart || warmStart.length !== horizon) return new Array(horizon).fill(previousU);
  return [...warmStart.slice(1), warmStart[warmStart.length - 1]];
}

export function solveBoxQPMPC(state, target, previousU, cfg, warmStart = null) {
  const started = now();
  const { A, B } = createSecondOrderModel(cfg);
  const qp = buildCondensedQP({ A, B, state, target, previousU, cfg });
  const horizon = cfg.mpc.horizon;
  const maxIterations = Math.max(1, Math.round(cfg.mpc.qpIterations ?? 120));
  const tolerance = Math.max(1e-12, cfg.mpc.qpTolerance ?? 1e-5);
  const lipschitz = maxRowAbsSum(qp.H);
  const step = (cfg.mpc.qpStepScale ?? 0.98) / lipschitz;

  let x = project(qp, shiftedWarmStart(warmStart, horizon, previousU));
  let y = [...x];
  let momentum = 1;
  let previousObjective = evaluateCondensedQP(qp, x);
  let finalResidual = kktResidual(qp, x);
  let converged = finalResidual <= tolerance;
  let iterations = 0;
  let restarts = 0;
  let objectiveImprovement = 0;

  for (let iter = 1; iter <= maxIterations && !converged; iter += 1) {
    const g = gradient(qp, y);
    let candidate = project(qp, y.map((value, i) => value - step * g[i]));
    let objective = evaluateCondensedQP(qp, candidate);

    // Monotone restart: acceleration is discarded if it raises the QP objective.
    if (objective > previousObjective + 1e-12) {
      const gx = gradient(qp, x);
      candidate = project(qp, x.map((value, i) => value - step * gx[i]));
      objective = evaluateCondensedQP(qp, candidate);
      y = [...x];
      momentum = 1;
      restarts += 1;
    }

    const nextMomentum = 0.5 * (1 + Math.sqrt(1 + 4 * momentum * momentum));
    const beta = (momentum - 1) / nextMomentum;
    const nextY = candidate.map((value, i) => value + beta * (value - x[i]));

    objectiveImprovement = previousObjective - objective;
    x = candidate;
    y = nextY;
    momentum = nextMomentum;
    previousObjective = objective;
    finalResidual = kktResidual(qp, x);
    converged = finalResidual <= tolerance;
    iterations = iter;
  }

  const feasibility = feasibilityViolation(qp, x);
  const active = activeConstraintStats(qp, x);
  const rollout = rolloutMPCSequence(state, x, target, previousU, cfg);

  return {
    u: x[0],
    sequence: x,
    path: rollout.xs.slice(1),
    cost: rollout.cost,
    qpObjective: evaluateCondensedQP(qp, x),
    solveMs: now() - started,
    solver: 'box-qp',
    diagnostics: {
      converged,
      iterations,
      kktResidual: finalResidual,
      feasibilityViolation: feasibility,
      activeConstraints: active.total,
      activeConstraintRatio: active.ratio,
      activeLower: active.lower,
      activeUpper: active.upper,
      lipschitzEstimate: lipschitz,
      stepSize: step,
      restarts,
      objectiveImprovement,
      finite: Number.isFinite(previousObjective) && Number.isFinite(finalResidual) && Number.isFinite(feasibility),
    },
  };
}

export function inspectBoxQPKKT(qp, x) {
  return {
    kktResidual: kktResidual(qp, x),
    feasibilityViolation: feasibilityViolation(qp, x),
    gradientInfinityNorm: maxAbs(gradient(qp, x)),
    active: activeConstraintStats(qp, x),
  };
}
