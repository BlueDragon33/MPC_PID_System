import {
  applySafetyGovernor,
  computeAdmissibleCommandInterval,
  computePhysicalCommandInterval,
} from './shortHorizonGovernor.js';

const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

export function computeGovernedPidCommand({
  pid,
  reference,
  controllerState,
  previousU,
  cfg,
  continuationSequence,
  lastMpcSafeU,
}) {
  const interval = computeAdmissibleCommandInterval(
    controllerState,
    previousU,
    cfg,
    continuationSequence,
  );
  const baseInterval = interval.baseInterval ?? computePhysicalCommandInterval(previousU, cfg);
  const pidResult = pid.updateDetailed(
    reference,
    controllerState.x,
    interval.feasible ? interval : null,
  );
  const pidRaw = pidResult.raw;
  const pidNominal = clamp(pidResult.raw, cfg.pid.uMin, cfg.pid.uMax);
  const pidConditioned = baseInterval.feasible
    ? clamp(pidNominal, baseInterval.lower, baseInterval.upper)
    : pidNominal;
  const conditioningCorrection = pidConditioned - pidNominal;

  if (interval.feasible) {
    const u = pidResult.u;
    const safetyCorrection = u - pidConditioned;
    return {
      u,
      pidRaw,
      pidNominal,
      pidConditioned,
      governor: {
        feasible: true,
        emergencyFallback: false,
        intervened: Math.abs(safetyCorrection) > 1e-12,
        conditioned: Math.abs(conditioningCorrection) > 1e-12,
        correction: safetyCorrection,
        conditioningCorrection,
        reason: Math.abs(safetyCorrection) > 1e-12
          ? 'safety-envelope-limited-command'
          : Math.abs(conditioningCorrection) > 1e-12
            ? 'actuator-plan-conditioned-command'
            : 'proposal-admissible',
        interval,
      },
    };
  }

  const fallbackGovernor = applySafetyGovernor({
    state: controllerState,
    proposedU: pidConditioned,
    previousU,
    lastMpcSafeU,
    continuationSequence,
    cfg,
  });
  const governor = {
    ...fallbackGovernor,
    intervened: true,
    conditioned: Math.abs(conditioningCorrection) > 1e-12,
    correction: fallbackGovernor.u - pidConditioned,
    conditioningCorrection,
  };

  return {
    u: governor.u,
    pidRaw,
    pidNominal,
    pidConditioned,
    governor,
  };
}
