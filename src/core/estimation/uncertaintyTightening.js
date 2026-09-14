const finiteNonNegative = (value) => Number.isFinite(value) ? Math.max(0, value) : 0;

export function covarianceSigmas(covariance, C = [1, 0]) {
  const P = covariance;
  if (!Array.isArray(P) || P.length !== 2 || !Array.isArray(P[0]) || !Array.isArray(P[1])) {
    return { positionSigma: 0, velocitySigma: 0, outputSigma: 0, valid: false };
  }

  const p00 = finiteNonNegative(P[0][0]);
  const p11 = finiteNonNegative(P[1][1]);
  const p01 = Number.isFinite(P[0][1]) ? P[0][1] : 0;
  const p10 = Number.isFinite(P[1][0]) ? P[1][0] : p01;
  const c0 = Number.isFinite(C?.[0]) ? C[0] : 1;
  const c1 = Number.isFinite(C?.[1]) ? C[1] : 0;
  const outputVariance = Math.max(0,
    c0 * c0 * p00
    + c0 * c1 * p01
    + c1 * c0 * p10
    + c1 * c1 * p11,
  );

  return {
    positionSigma: Math.sqrt(p00),
    velocitySigma: Math.sqrt(p11),
    outputSigma: Math.sqrt(outputVariance),
    valid: Number.isFinite(p00) && Number.isFinite(p11) && Number.isFinite(outputVariance),
  };
}

export function computeConstraintTightening(cfg, covariance, C = [1, 0]) {
  const enabled = Boolean(cfg.estimation?.enabled && cfg.estimation?.constraintTighteningEnabled);
  const sigmaMultiplier = Math.max(0, Number.isFinite(cfg.estimation?.constraintSigma)
    ? cfg.estimation.constraintSigma
    : 0);
  const sigmas = covarianceSigmas(covariance, C);

  if (!enabled || sigmaMultiplier <= 0 || !sigmas.valid) {
    return {
      enabled: false,
      sigmaMultiplier,
      positionMargin: 0,
      velocityMargin: 0,
      outputMargin: 0,
      ...sigmas,
    };
  }

  return {
    enabled: true,
    sigmaMultiplier,
    positionMargin: sigmaMultiplier * sigmas.positionSigma,
    velocityMargin: sigmaMultiplier * sigmas.velocitySigma,
    outputMargin: sigmaMultiplier * sigmas.outputSigma,
    ...sigmas,
  };
}

export function applyCovarianceConstraintTightening(cfg, covariance, C = [1, 0]) {
  const tightening = computeConstraintTightening(cfg, covariance, C);
  if (!tightening.enabled) return { config: cfg, tightening: { ...tightening, validEnvelope: true } };

  const mpc = { ...cfg.mpc };
  if (mpc.stateConstraintsEnabled) {
    if (Number.isFinite(mpc.positionMin)) mpc.positionMin += tightening.positionMargin;
    if (Number.isFinite(mpc.positionMax)) mpc.positionMax -= tightening.positionMargin;
    if (Number.isFinite(mpc.velocityMin)) mpc.velocityMin += tightening.velocityMargin;
    if (Number.isFinite(mpc.velocityMax)) mpc.velocityMax -= tightening.velocityMargin;
  }
  if (mpc.outputConstraintsEnabled) {
    if (Number.isFinite(mpc.outputMin)) mpc.outputMin += tightening.outputMargin;
    if (Number.isFinite(mpc.outputMax)) mpc.outputMax -= tightening.outputMargin;
  }

  const positionValid = !mpc.stateConstraintsEnabled
    || !(Number.isFinite(mpc.positionMin) && Number.isFinite(mpc.positionMax))
    || mpc.positionMin <= mpc.positionMax;
  const velocityValid = !mpc.stateConstraintsEnabled
    || !(Number.isFinite(mpc.velocityMin) && Number.isFinite(mpc.velocityMax))
    || mpc.velocityMin <= mpc.velocityMax;
  const outputValid = !mpc.outputConstraintsEnabled
    || !(Number.isFinite(mpc.outputMin) && Number.isFinite(mpc.outputMax))
    || mpc.outputMin <= mpc.outputMax;

  return {
    config: { ...cfg, mpc },
    tightening: {
      ...tightening,
      validEnvelope: positionValid && velocityValid && outputValid,
      positionValid,
      velocityValid,
      outputValid,
    },
  };
}
