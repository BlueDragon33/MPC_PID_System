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

function sparseDot(row, x) {
  let sum = 0;
  for (let k = 0; k < row.indices.length; k += 1) {
    sum += row.values[k] * x[row.indices[k]];
  }
  return sum;
}

function repairTowardFeasibleAnchor(inequalities, anchor, candidate, tolerance) {
  const anchorViolation = inequalityViolation(inequalities, anchor);
  if (anchorViolation.maxViolation > tolerance) return null;

  const direction = candidate.map((value, i) => value - anchor[i]);
  let alpha = 1;
  for (const row of inequalities.rows) {
    const base = sparseDot(row, anchor);
    let directional = 0;
    for (let k = 0; k < row.indices.length; k += 1) {
      directional += row.values[k] * direction[row.indices[k]];
    }
    if (directional <= 0) continue;
    alpha = Math.min(alpha, Math.max(0, (row.bound - base) / directional));
  }

  // Step infinitesimally inside the polyhedron to absorb floating-point error.
  alpha = Math.max(0, Math.min(1, alpha * (1 - 1e-12)));
  const x = anchor.map((value, i) => value + alpha * direction[i]);
  return {
    x,
    alpha,
    feasibility: inequalityViolation(inequalities, x),
  };
}

function project(qp, x, options, feasibleAnchor = null) {
  const projected = projectPolyhedronDykstra(qp.inequalities, x, {
    maxCycles: options.projectionCycles,
    tolerance: options.projectionTolerance,
  });
  if (projected.maxViolation <= options.feasibilityTolerance) {
    return { ...projected, anchorRepairUsed: false, anchorAlpha: 1 };
  }
  if (!feasibleAnchor) return { ...projected, anchorRepairUsed: false, anchorAlpha: null };

  const repaired = repairTowardFeasibleAnchor(
    qp.inequalities,
    feasibleAnchor,
    projected.x,
    options.feasibilityTolerance,
  );
  if (!repaired) return { ...projected, anchorRepairUsed: false, anchorAlpha: null };
  return {
    ...projected,
    x: repaired.x,
    maxViolation: repaired.feasibility.maxViolation,
    violated: repaired.feasibility.violated,
    worstViolation: repaired.feasibility.worst,
    converged: repaired.feasibility.maxViolation <= options.feasibilityTolerance,
    anchorRepairUsed: true,
    anchorAlpha: repaired.alpha,
  };
}

function projectedGradientResidual(qp, x, step, options) {
  const g = gradient(qp, x);
  const trial = x.map((value, i) => value - step * g[i]);
  const projected = project(qp, trial, options, x);
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

  const zeroSeed = new Array(dimension).fill(0);
  const seed = warmStart?.length === dimension ? [...warmStart] : [...zeroSeed];
  const candidateAnchors = [
    seed,
    options.feasibleSeed?.length === dimension ? [...options.feasibleSeed] : null,
    zeroSeed,
  ].filter(Boolean);
  const feasibleSeed = candidateAnchors.find(
    (candidate) => inequalityViolation(qp.inequalities, candidate).maxViolation <= opts.feasibilityTolerance,
  ) ?? null;
  const initial = project(qp, seed, opts, feasibleSeed);
  let x = initial.x;
  let feasibleAnchor = initial.maxViolation <= opts.feasibilityTolerance ? [...x] : feasibleSeed;
  let anchorRepairs = initial.anchorRepairUsed ? 1 : 0;

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
  let y = [...x];
  let momentum = 1;
  let restarts = 0;
  let residualInfo = projectedGradientResidual(qp, x, step, opts);
  let feasibility = inequalityViolation(qp.inequalities, x);
  let converged = residualInfo.residual <= opts.tolerance
    && feasibility.maxViolation <= opts.feasibilityTolerance;
  let iterations = 0;
  let projectionCycles = initial.cycles + residualInfo.projection.cycles;
  let previousObjective = objective(qp, x);

  for (let iter = 1; iter <= opts.maxIterations && !converged; iter += 1) {
    const previousX = [...x];
    const g = gradient(qp, y);
    let projected = project(
      qp,
      y.map((value, i) => value - step * g[i]),
      opts,
      feasibleAnchor,
    );
    projectionCycles += projected.cycles;
    if (projected.anchorRepairUsed) anchorRepairs += 1;
    let candidate = projected.x;
    let candidateObjective = objective(qp, candidate);

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

    if (candidateObjective > previousObjective + 1e-12) {
      const restartGradient = gradient(qp, previousX);
      projected = project(
        qp,
        previousX.map((value, i) => value - step * restartGradient[i]),
        opts,
        feasibleAnchor,
      );
      projectionCycles += projected.cycles;
      if (projected.anchorRepairUsed) anchorRepairs += 1;
      candidate = projected.x;
      candidateObjective = objective(qp, candidate);
      y = [...previousX];
      momentum = 1;
      restarts += 1;
    }

    x = candidate;
    if (inequalityViolation(qp.inequalities, x).maxViolation <= opts.feasibilityTolerance) {
      feasibleAnchor = [...x];
    }

    const nextMomentum = 0.5 * (1 + Math.sqrt(1 + 4 * momentum * momentum));
    const beta = (momentum - 1) / nextMomentum;
    y = x.map((value, i) => value + beta * (value - previousX[i]));
    momentum = nextMomentum;
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
      anchorRepairs,
      restarts,
    },
  };
}
