export const defaultUgvConfig = Object.freeze({
  dt: 0.05,
  duration: 14,
  ugv: {
    wheelbase: 2.7,
    accelerationMin: -3.0,
    accelerationMax: 2.0,
    accelerationRateMin: -4.0,
    accelerationRateMax: 4.0,
    steeringMin: -0.55,
    steeringMax: 0.55,
    steeringRateMin: -0.7,
    steeringRateMax: 0.7,
    speedMin: 0,
    speedMax: 10,
  },
  ugvPath: {
    amplitude: 1.2,
    waveNumber: 0.075,
    referenceSpeed: 5.0,
  },
  ugvSafety: {
    laneHalfWidth: 1.5,
    speedMin: 0,
    speedMax: 8,
  },
  ugvClassical: {
    stanleyGain: 1.6,
    softeningSpeed: 1.2,
    curvatureFeedforward: 1,
    speedKp: 1.1,
    speedKi: 0.28,
    speedKd: 0.08,
    speedIntegralLimit: 4,
  },
  ugvEstimation: {
    enabled: true,
    seed: 20260928,
    processVariance: [2e-4, 2e-4, 5e-5, 8e-3],
    measurementStd: [0.06, 0.06, 0.012, 0.08],
    initialVariance: [0.12, 0.12, 0.03, 0.3],
  },
  ugvLtvMpc: {
    horizon: 14,
    qLateral: 16,
    qHeading: 11,
    qSpeed: 2.2,
    terminalScale: 4,
    rAcceleration: 0.16,
    rSteering: 1.3,
    rDeltaAcceleration: 0.2,
    rDeltaSteering: 2.4,
    qpIterations: 160,
    qpTolerance: 2e-5,
    qpFeasibilityTolerance: 1e-7,
    qpProjectionCycles: 24,
    qpProjectionTolerance: 1e-10,
    qpStepScale: 0.9,
  },
});

function mergeGroup(base, patch) {
  return { ...base, ...(patch || {}) };
}

export function mergeUgvConfig(userConfig = {}) {
  return {
    ...defaultUgvConfig,
    ...userConfig,
    ugv: mergeGroup(defaultUgvConfig.ugv, userConfig.ugv),
    ugvPath: mergeGroup(defaultUgvConfig.ugvPath, userConfig.ugvPath),
    ugvSafety: mergeGroup(defaultUgvConfig.ugvSafety, userConfig.ugvSafety),
    ugvClassical: mergeGroup(defaultUgvConfig.ugvClassical, userConfig.ugvClassical),
    ugvEstimation: mergeGroup(defaultUgvConfig.ugvEstimation, userConfig.ugvEstimation),
    ugvLtvMpc: mergeGroup(defaultUgvConfig.ugvLtvMpc, userConfig.ugvLtvMpc),
  };
}
