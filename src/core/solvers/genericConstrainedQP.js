import {
  activeInequalityStats,
  inequalityViolation,
  projectPolyhedronDykstra,
} from '../mpc/constraints.js';

const now = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

function matVec(A, x) {
  return A.map((row) => row.reduce((sum, value, j) => sum + value * x[j], 0));
}

function gradient(qp, x) {
  const Hx = matVec(qp.H, x);
  return Hx.map((value, i) => value + qp.f[i]);
}

function objective(qp, x) {
  const Hx = matVec(qp.H, x);
  return 0.5 * x.reduce((sum, value, i) => sum + value * Hx[i], 0)
    + x.reduce((sum, value, i) => sum + value * qp.f[i], 0);
}

function maxAbs(values) {
  return values.reduce((max, value) => Math.max(max, Math.abs(value)), 0);
}

function maxRowAbsSum(H) {
  return Math.max(
    1e-9,
    ...H.map((row) => row.reduce((sum, value) => sum + Math.abs(value), 0)),
  );
}

function project(qp, x, options) {
  return projectPolyhedronDykstra(qp.inequalities, x, {
    maxCycles: options.projectionCycles,
    tolerance: options.projectionTolerance,
  });
}

function projectedGradientResidual(qp, x, step, options) {
  const g = gradient(qp, x);
  const trial = x.map((value, i) => value - step * g[i]);
  const projected = project(qp, trial, options);
  return {
    residual: maxAbs(x.map((value, i) => (value - projected.x[i]) / step)),
    projection: projected,
  };
}

export function solveGenericConstrainedQP(qp, options = {}, warmStart = null) {
  const started = now();
  const dimension = qp.H.length;
  const opts = {
    maxIterations: Math.max(1, Math.round(options.maxIterations ?? 160)),
    tolerance: Math.max(1e-12, options.tolerance ?? 2e-5),
    feasibilityTolerance: Math.max(1e-12, options.feasibilityTolerance ?? 1e-7),
    projectionCycles: Math.max(1, Math.round(options.projectionCycles ?? 24)),
    projectionTolerance: Math.max(1e-14, options.projectionTolerance ?? 1e-10),
    stepScale: Math.max(1e-4, options.stepScale ?? 0.9),
  };

  const seed = warmStart?.length === dimension
    ? [...warmStart]
    : new Array(dimension).fill(0);
  const initial = project(qp, seed, opts);
  let x = initial.x;

  if (initial.maxViolation > opts.feasibilityTolerance * 10) {
    return {
      x,
      status: 'infeasible',
      solveMs: now() - started,
      objective: objective(qp, x),
      diagnostics: {
        converged: false,
        iterations: 0,
        feasibilityViolation: initial.maxViolation,
        worstViolation: initial.worstViolation,
        projectionCycles: initial.cycles,
        finite: x.every(Number.isFinite),
      },
    };
  }

  const lipschitz = maxRowAbsSum(qp.H);
  const step = opts.stepScale / lipschitz;
  let residualInfo = projectedGradientResidual(qp, x, step, opts);
  let feasibility = inequalityViolation(qp.inequalities, x);
  let converged = residualInfo.residual <= opts.tolerance
    && feasibility.maxViolation <= opts.feasibilityTolerance;
  let iterations = 0;
  let projectionCycles = initial.cycles + residualInfo.projection.cycles;
  let previousObjective = objective(qp, x);

  for (let iter = 1; iter <= opts.maxIterations && !converged; iter += 1) {
    const g = gradient(qp, x);
    const projected = project(qp, x.map((value, i) => value - step * g[i]), opts);
    projectionCycles += projected.cycles;
    const candidate = projected.x;
    const candidateObjective = objective(qp, candidate);

    if (!candidate.every(Number.isFinite) || !Number.isFinite(candidateObjective)) {
      return {
        x,
        status: 'numerical-failure',
        solveMs: now() - started,
        objective: previousObjective,
        diagnostics: {
          converged: false,
          iterations: iter,
          feasibilityViolation: inequalityViolation(qp.inequalities, x).maxViolation,
          projectionCycles,
          finite: false,
        },
      };
    }

    x = candidate;
    previousObjective = candidateObjective;
    residualInfo = projectedGradientResidual(qp, x, step, opts);
    projectionCycles += residualInfo.projection.cycles;
    feasibility = inequalityViolation(qp.inequalities, x);
    converged = residualInfo.residual <= opts.tolerance
      && feasibility.maxViolation <= opts.feasibilityTolerance;
    iterations = iter;
  }

  const active = activeInequalityStats(qp.inequalities, x);
  feasibility = inequalityViolation(qp.inequalities, x);
  const feasible = feasibility.maxViolation <= opts.feasibilityTolerance * 10;

  return {
    x,
    status: feasible ? (converged ? 'solved' : 'max-iterations') : 'infeasible',
    solveMs: now() - started,
    objective: previousObjective,
    diagnostics: {
      converged,
      acceptedApproximate: feasible && !converged,
      iterations,
      projectedGradientResidual: residualInfo.residual,
      feasibilityViolation: feasibility.maxViolation,
      worstViolation: feasibility.worst,
      activeConstraints: active.total,
      activeConstraintRatio: active.ratio,
      activeByKind: active.byKind,
      projectionCycles,
      lipschitzEstimate: lipschitz,
      stepSize: step,
      finite: x.every(Number.isFinite) && Number.isFinite(previousObjective),
    },
  };
}
