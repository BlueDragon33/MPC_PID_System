import { createSecondOrderModel } from '../models/secondOrderPlant.js';

const EPS = 1e-12;
const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

function matVec2(A, x) {
  return [
    A[0][0] * x[0] + A[0][1] * x[1],
    A[1][0] * x[0] + A[1][1] * x[1],
  ];
}

function add2(a, b) {
  return [a[0] + b[0], a[1] + b[1]];
}

function scale2(v, scalar) {
  return [v[0] * scalar, v[1] * scalar];
}

function intersectAffineEnvelope(interval, freeValue, coefficient, minimum, maximum, label, stage) {
  let { lower, upper } = interval;

  if (Math.abs(coefficient) <= EPS) {
    if (Number.isFinite(minimum) && freeValue < minimum - EPS) {
      return { ...interval, feasible: false, reason: `${label}-lower-unreachable`, stage, freeValue };
    }
    if (Number.isFinite(maximum) && freeValue > maximum + EPS) {
      return { ...interval, feasible: false, reason: `${label}-upper-unreachable`, stage, freeValue };
    }
    return interval;
  }

  if (Number.isFinite(minimum)) {
    const boundary = (minimum - freeValue) / coefficient;
    if (coefficient > 0) lower = Math.max(lower, boundary);
    else upper = Math.min(upper, boundary);
  }
  if (Number.isFinite(maximum)) {
    const boundary = (maximum - freeValue) / coefficient;
    if (coefficient > 0) upper = Math.min(upper, boundary);
    else lower = Math.max(lower, boundary);
  }

  return {
    lower,
    upper,
    feasible: lower <= upper + EPS,
    reason: lower <= upper + EPS ? 'ok' : `${label}-interval-empty`,
    stage,
  };
}

function physicalCommandInterval(previousU, cfg) {
  let lower = cfg.mpc.uMin;
  let upper = cfg.mpc.uMax;
  if (Number.isFinite(cfg.mpc.deltaUMin)) lower = Math.max(lower, previousU + cfg.mpc.deltaUMin);
  if (Number.isFinite(cfg.mpc.deltaUMax)) upper = Math.min(upper, previousU + cfg.mpc.deltaUMax);
  return {
    lower,
    upper,
    feasible: lower <= upper + EPS,
    reason: lower <= upper + EPS ? 'ok' : 'actuator-rate-interval-empty',
    stage: 0,
  };
}

function intersectFirstMoveWithContinuationRate(interval, continuationSequence, cfg) {
  if (!continuationSequence || continuationSequence.length < 2 || !Number.isFinite(continuationSequence[1])) return interval;
  const nextPlannedU = continuationSequence[1];
  let lower = interval.lower;
  let upper = interval.upper;

  // Future planned move must remain reachable from the PID/governor first move.
  // deltaUMin <= u1 - u0 <= deltaUMax
  if (Number.isFinite(cfg.mpc.deltaUMax)) lower = Math.max(lower, nextPlannedU - cfg.mpc.deltaUMax);
  if (Number.isFinite(cfg.mpc.deltaUMin)) upper = Math.min(upper, nextPlannedU - cfg.mpc.deltaUMin);

  return {
    ...interval,
    lower,
    upper,
    feasible: lower <= upper + EPS,
    reason: lower <= upper + EPS ? interval.reason : 'first-move-breaks-mpc-continuation-rate',
  };
}

function plannedFutureInput(continuationSequence, stage) {
  if (!continuationSequence?.length) return null;
  const index = Math.min(stage - 1, continuationSequence.length - 1);
  const value = continuationSequence[index];
  return Number.isFinite(value) ? value : null;
}

export function computeAdmissibleCommandInterval(state, previousU, cfg, continuationSequence = null) {
  const horizon = Math.max(1, Math.round(cfg.safety?.previewHorizon ?? 6));
  const { A, B, C } = createSecondOrderModel(cfg);
  const hasContinuation = Array.isArray(continuationSequence) && continuationSequence.length > 1;
  let interval = physicalCommandInterval(previousU, cfg);
  if (!interval.feasible) return { ...interval, horizon, checkedStages: 0, continuationUsed: false };

  if (hasContinuation) {
    interval = intersectFirstMoveWithContinuationRate(interval, continuationSequence, cfg);
    if (!interval.feasible) return { ...interval, horizon, checkedStages: 0, continuationUsed: true };
  }

  const enforceState = Boolean(cfg.mpc.stateConstraintsEnabled);
  const enforceOutput = Boolean(cfg.mpc.outputConstraintsEnabled);
  if (!enforceState && !enforceOutput) return { ...interval, horizon, checkedStages: 0, continuationUsed: hasContinuation };

  const positionMargin = Math.max(0, cfg.safety?.positionMargin ?? 0);
  const velocityMargin = Math.max(0, cfg.safety?.velocityMargin ?? 0);
  const outputMargin = Math.max(0, cfg.safety?.outputMargin ?? 0);

  // State prediction is affine in the candidate first move:
  // x_h = free_h + gain_h * u0.
  let free = matVec2(A, [state.x, state.v]);
  let gain = [...B];

  for (let stage = 1; stage <= horizon; stage += 1) {
    if (stage > 1) {
      if (hasContinuation) {
        const futureU = plannedFutureInput(continuationSequence, stage);
        free = add2(matVec2(A, free), scale2(B, futureU ?? previousU));
        gain = matVec2(A, gain);
      } else {
        // No MPC continuation is available. Fall back to the conservative
        // constant-command preview used by the first governor prototype.
        free = matVec2(A, free);
        gain = add2(matVec2(A, gain), B);
      }
    }

    if (enforceState) {
      interval = intersectAffineEnvelope(
        interval,
        free[0],
        gain[0],
        Number.isFinite(cfg.mpc.positionMin) ? cfg.mpc.positionMin + positionMargin : null,
        Number.isFinite(cfg.mpc.positionMax) ? cfg.mpc.positionMax - positionMargin : null,
        'position',
        stage,
      );
      if (!interval.feasible) return { ...interval, horizon, checkedStages: stage, continuationUsed: hasContinuation };

      interval = intersectAffineEnvelope(
        interval,
        free[1],
        gain[1],
        Number.isFinite(cfg.mpc.velocityMin) ? cfg.mpc.velocityMin + velocityMargin : null,
        Number.isFinite(cfg.mpc.velocityMax) ? cfg.mpc.velocityMax - velocityMargin : null,
        'velocity',
        stage,
      );
      if (!interval.feasible) return { ...interval, horizon, checkedStages: stage, continuationUsed: hasContinuation };
    }

    if (enforceOutput) {
      const freeY = C[0] * free[0] + C[1] * free[1];
      const gainY = C[0] * gain[0] + C[1] * gain[1];
      interval = intersectAffineEnvelope(
        interval,
        freeY,
        gainY,
        Number.isFinite(cfg.mpc.outputMin) ? cfg.mpc.outputMin + outputMargin : null,
        Number.isFinite(cfg.mpc.outputMax) ? cfg.mpc.outputMax - outputMargin : null,
        'output',
        stage,
      );
      if (!interval.feasible) return { ...interval, horizon, checkedStages: stage, continuationUsed: hasContinuation };
    }
  }

  return { ...interval, horizon, checkedStages: horizon, continuationUsed: hasContinuation };
}

export function applySafetyGovernor({ state, proposedU, previousU, lastMpcSafeU, continuationSequence = null, cfg }) {
  const interval = computeAdmissibleCommandInterval(state, previousU, cfg, continuationSequence);
  const physical = physicalCommandInterval(previousU, cfg);

  if (!interval.feasible) {
    const fallbackCandidate = Number.isFinite(lastMpcSafeU) ? lastMpcSafeU : previousU;
    const fallbackU = physical.feasible
      ? clamp(fallbackCandidate, physical.lower, physical.upper)
      : previousU;
    return {
      u: fallbackU,
      proposedU,
      correction: fallbackU - proposedU,
      intervened: Math.abs(fallbackU - proposedU) > 1e-12,
      feasible: false,
      emergencyFallback: true,
      reason: interval.reason,
      interval,
    };
  }

  const safeU = clamp(proposedU, interval.lower, interval.upper);
  return {
    u: safeU,
    proposedU,
    correction: safeU - proposedU,
    intervened: Math.abs(safeU - proposedU) > 1e-12,
    feasible: true,
    emergencyFallback: false,
    reason: Math.abs(safeU - proposedU) > 1e-12 ? 'projected-to-admissible-interval' : 'proposal-admissible',
    interval,
  };
}
