import { createSecondOrderModel } from '../models/secondOrderPlant.js';
import { buildCondensedQP, evaluateCondensedQP } from '../mpc/condensedQP.js';
import {
  activeInequalityStats,
  inequalityViolation,
  precheckInputRateFeasibility,
  projectPolyhedronDykstra,
  repairInputRateFeasibility,
} from '../mpc/constraints.js';
import { rolloutMPCSequence } from '../mpc/rollout.js';

const now = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

function matVec(A, x) {
  return A.map((row) => row.reduce((sum, value, j) => sum + value * x[j], 0));
}

function gradient(qp, x) {
  const Hx = matVec(qp.H, x);
  return Hx.map((value, i) => value + qp.f[i]);
}

function maxAbs(values) {
  let result = 0;
  for (const value of values) result = Math.max(result, Math.abs(value));
  return result;
}

function maxRowAbsSum(H) {
  let result = 0;
  for (const row of H) result = Math.max(result, row.reduce((sum, value) => sum + Math.abs(value), 0));
  return Math.max(result, 1e-9);
}

function shiftedWarmStart(warmStart, horizon, previousU) {
  if (!warmStart || warmStart.length !== horizon) return new Array(horizon).fill(previousU);
  return [...warmStart.slice(1), warmStart[warmStart.length - 1]];
}

function project(qp, x, cfg) {
  return projectPolyhedronDykstra(qp.inequalities, x, {
    maxCycles: cfg.mpc.qpProjectionCycles ?? 10,
    tolerance: cfg.mpc.qpProjectionTolerance ?? 1e-10,
  });
}

function projectedGradientResidual(qp, x, step, cfg) {
  const g = gradient(qp, x);
  const trial = x.map((value, i) => value - step * g[i]);
  const projected = project(qp, trial, cfg);
  const mapping = x.map((value, i) => (value - projected.x[i]) / step);
  return {
    residual: maxAbs(mapping),
    projection: projected,
  };
}

function finiteVector(x) {
  return x.every(Number.isFinite);
}

function fallbackSequence(qp, previousU, cfg) {
  // Safety hierarchy: actuator magnitude and slew-rate limits are physical hard
  // constraints. If a predicted state/output envelope is infeasible, fallback
  // keeps the actuator plan valid and reports the remaining envelope violation.
  const seed = new Array(cfg.mpc.horizon).fill(previousU);
  const repaired = repairInputRateFeasibility(qp.inequalities, seed, cfg.mpc.qpFeasibilityTolerance ?? 1e-7);
  const sequence = repaired.x;
  const overall = inequalityViolation(qp.inequalities, sequence);
  return {
    sequence,
    actuatorFeasible: repaired.feasible,
    allConstraintsFeasible: overall.maxViolation <= (cfg.mpc.qpFeasibilityTolerance ?? 1e-7),
    overallViolation: overall,
  };
}

function buildFailureResult({ qp, state, target, previousU, cfg, started, status, reason, diagnostics = {} }) {
  const fallback = fallbackSequence(qp, previousU, cfg);
  const sequence = fallback.sequence;
  const rollout = rolloutMPCSequence(state, sequence, target, previousU, cfg);
  const feasibility = inequalityViolation(qp.inequalities, sequence);

  return {
    u: sequence[0],
    sequence,
    path: rollout.xs.slice(1),
    cost: rollout.cost,
    qpObjective: evaluateCondensedQP(qp, sequence),
    solveMs: now() - started,
    solver: 'constrained-qp',
    status,
    fallbackUsed: true,
    fallbackReason: reason,
    diagnostics: {
      converged: false,
      acceptedApproximate: false,
      iterations: diagnostics.iterations ?? 0,
      projectedGradientResidual: diagnostics.projectedGradientResidual ?? Number.POSITIVE_INFINITY,
      feasibilityViolation: feasibility.maxViolation,
      violatedConstraints: feasibility.violated,
      worstViolation: feasibility.worst,
      activeConstraints: activeInequalityStats(qp.inequalities, sequence).total,
      activeConstraintRatio: activeInequalityStats(qp.inequalities, sequence).ratio,
      projectionCycles: diagnostics.projectionCycles ?? 0,
      restarts: diagnostics.restarts ?? 0,
      finite: finiteVector(sequence),
      disturbanceCompensationEnabled: Boolean(qp.disturbanceCompensationEnabled),
      fallbackActuatorFeasible: fallback.actuatorFeasible,
      fallbackAllConstraintsFeasible: fallback.allConstraintsFeasible,
      precheck: diagnostics.precheck ?? null,
    },
  };
}

export function solveConstrainedQPMPC(state, target, previousU, cfg, warmStart = null) {
  const started = now();
  const { A, B, E, C } = createSecondOrderModel(cfg);
  const qp = buildCondensedQP({ A, B, E, C, state, target, previousU, cfg });
  const precheck = precheckInputRateFeasibility(qp.inequalities, previousU, cfg.mpc.uMin, cfg.mpc.uMax);

  if (!precheck.feasible) {
    return buildFailureResult({
      qp, state, target, previousU, cfg, started,
      status: 'infeasible',
      reason: precheck.reason,
      diagnostics: { precheck },
    });
  }

  const maxIterations = Math.max(1, Math.round(cfg.mpc.qpIterations ?? 120));
  const tolerance = Math.max(1e-12, cfg.mpc.qpTolerance ?? 1e-5);
  const feasibilityTolerance = Math.max(1e-12, cfg.mpc.qpFeasibilityTolerance ?? 1e-7);
  const timeBudgetMs = Math.max(0, cfg.mpc.qpTimeBudgetMs ?? 0);
  const lipschitz = maxRowAbsSum(qp.H);
  const step = (cfg.mpc.qpStepScale ?? 0.95) / lipschitz;

  const initialProjection = project(qp, shiftedWarmStart(warmStart, cfg.mpc.horizon, previousU), cfg);
  let x = initialProjection.x;
  let y = [...x];
  let momentum = 1;
  let previousObjective = evaluateCondensedQP(qp, x);
  let residualInfo = projectedGradientResidual(qp, x, step, cfg);
  let converged = residualInfo.residual <= tolerance && inequalityViolation(qp.inequalities, x).maxViolation <= feasibilityTolerance;
  let iterations = 0;
  let restarts = 0;
  let projectionCycles = initialProjection.cycles + residualInfo.projection.cycles;
  let timedOut = false;

  for (let iter = 1; iter <= maxIterations && !converged; iter += 1) {
    if (timeBudgetMs > 0 && now() - started >= timeBudgetMs) {
      timedOut = true;
      break;
    }

    const g = gradient(qp, y);
    const candidateProjection = project(qp, y.map((value, i) => value - step * g[i]), cfg);
    projectionCycles += candidateProjection.cycles;
    let candidate = candidateProjection.x;
    let objective = evaluateCondensedQP(qp, candidate);

    if (!Number.isFinite(objective) || !finiteVector(candidate)) {
      return buildFailureResult({
        qp, state, target, previousU, cfg, started,
        status: 'numerical-failure',
        reason: 'non-finite-candidate',
        diagnostics: { iterations: iter, projectionCycles, restarts, precheck },
      });
    }

    if (objective > previousObjective + 1e-12) {
      const gx = gradient(qp, x);
      const restartProjection = project(qp, x.map((value, i) => value - step * gx[i]), cfg);
      projectionCycles += restartProjection.cycles;
      candidate = restartProjection.x;
      objective = evaluateCondensedQP(qp, candidate);
      y = [...x];
      momentum = 1;
      restarts += 1;
    }

    const nextMomentum = 0.5 * (1 + Math.sqrt(1 + 4 * momentum * momentum));
    const beta = (momentum - 1) / nextMomentum;
    const nextY = candidate.map((value, i) => value + beta * (value - x[i]));

    x = candidate;
    y = nextY;
    momentum = nextMomentum;
    previousObjective = objective;
    residualInfo = projectedGradientResidual(qp, x, step, cfg);
    projectionCycles += residualInfo.projection.cycles;
    const feasibility = inequalityViolation(qp.inequalities, x).maxViolation;
    converged = residualInfo.residual <= tolerance && feasibility <= feasibilityTolerance;
    iterations = iter;
  }

  if (timedOut) {
    return buildFailureResult({
      qp, state, target, previousU, cfg, started,
      status: 'timeout',
      reason: 'qp-time-budget-exceeded',
      diagnostics: {
        iterations,
        projectedGradientResidual: residualInfo.residual,
        projectionCycles,
        restarts,
        precheck,
      },
    });
  }

  const feasibility = inequalityViolation(qp.inequalities, x);
  if (feasibility.maxViolation > feasibilityTolerance * 10) {
    return buildFailureResult({
      qp, state, target, previousU, cfg, started,
      status: 'infeasible',
      reason: feasibility.worst ? `constraint-unreachable:${feasibility.worst.kind}@${feasibility.worst.stage}` : 'projection-could-not-reach-feasible-set',
      diagnostics: {
        iterations,
        projectedGradientResidual: residualInfo.residual,
        projectionCycles,
        restarts,
        precheck,
      },
    });
  }

  const status = converged ? 'solved' : 'max-iterations';
  const acceptedApproximate = !converged && feasibility.maxViolation <= feasibilityTolerance;
  const active = activeInequalityStats(qp.inequalities, x);
  const rollout = rolloutMPCSequence(state, x, target, previousU, cfg);

  return {
    u: x[0],
    sequence: x,
    path: rollout.xs.slice(1),
    cost: rollout.cost,
    qpObjective: evaluateCondensedQP(qp, x),
    solveMs: now() - started,
    solver: 'constrained-qp',
    status,
    fallbackUsed: false,
    fallbackReason: null,
    diagnostics: {
      converged,
      acceptedApproximate,
      iterations,
      projectedGradientResidual: residualInfo.residual,
      feasibilityViolation: feasibility.maxViolation,
      violatedConstraints: feasibility.violated,
      worstViolation: feasibility.worst,
      activeConstraints: active.total,
      activeConstraintRatio: active.ratio,
      activeByKind: active.byKind,
      stateConstraintsEnabled: qp.inequalities.stateConstraintsEnabled,
      outputConstraintsEnabled: qp.inequalities.outputConstraintsEnabled,
      stateConstraintCount: qp.inequalities.stateConstraintCount || 0,
      outputConstraintCount: qp.inequalities.outputConstraintCount || 0,
      disturbanceCompensationEnabled: Boolean(qp.disturbanceCompensationEnabled),
      lipschitzEstimate: lipschitz,
      stepSize: step,
      projectionCycles,
      restarts,
      finite: Number.isFinite(previousObjective) && Number.isFinite(residualInfo.residual) && finiteVector(x),
      precheck,
    },
  };
}
