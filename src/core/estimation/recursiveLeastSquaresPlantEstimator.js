const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;
const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

function dot(a, b) {
  return a.reduce((sum, value, i) => sum + value * b[i], 0);
}

function matVec(A, x) {
  return A.map((row) => dot(row, x));
}

function identity3(scale = 1) {
  return [[scale, 0, 0], [0, scale, 0], [0, 0, scale]];
}

function covariance3(value, fallbackScale) {
  if (Array.isArray(value) && value.length === 3 && value.every((row) => Array.isArray(row) && row.length === 3)) {
    return value.map((row, i) => row.map((entry, j) => finite(entry, i === j ? fallbackScale : 0)));
  }
  if (Array.isArray(value) && value.length === 3) {
    return [[finite(value[0], fallbackScale), 0, 0], [0, finite(value[1], fallbackScale), 0], [0, 0, finite(value[2], fallbackScale)]];
  }
  return identity3(fallbackScale);
}

function symmetrize(P) {
  return P.map((row, i) => row.map((value, j) => {
    const symmetric = 0.5 * (finite(value) + finite(P[j][i]));
    return i === j ? Math.max(1e-12, symmetric) : symmetric;
  }));
}

function normalizedBounds(bounds = {}) {
  return {
    stiffness: [
      finite(bounds.stiffnessMin, 0.05),
      finite(bounds.stiffnessMax, 10),
    ],
    damping: [
      finite(bounds.dampingMin, 0.01),
      finite(bounds.dampingMax, 10),
    ],
    gain: [
      finite(bounds.gainMin, 0.05),
      finite(bounds.gainMax, 10),
    ],
  };
}

export function createRecursiveLeastSquaresPlantEstimator({
  dt,
  initialParameters = { stiffness: 1, damping: 1, gain: 1 },
  initialCovariance = 100,
  forgettingFactor = 0.995,
  bounds = {},
  minimumRegressorNorm = 1e-6,
} = {}) {
  const sampleTime = Math.max(1e-9, finite(dt, 0.02));
  const lambda = clamp(finite(forgettingFactor, 0.995), 0.9, 1);
  const parameterBounds = normalizedBounds(bounds);
  let theta = [
    finite(initialParameters.stiffness, 1),
    finite(initialParameters.damping, 1),
    finite(initialParameters.gain, 1),
  ];
  let P = covariance3(initialCovariance, 100);
  let updates = 0;
  let skippedUpdates = 0;
  let residualSqSum = 0;
  let lastDiagnostics = null;

  function update({ previousState, nextState, u, disturbanceEstimate = 0 }) {
    const x = finite(previousState?.x);
    const v = finite(previousState?.v);
    const nextV = finite(nextState?.v, v);
    const command = finite(u);
    const disturbance = finite(disturbanceEstimate);
    const phi = [-x, -v, command];
    const regressorNormSq = dot(phi, phi);
    const measuredAccelerationWithoutDisturbance = (nextV - v) / sampleTime - disturbance;
    const prediction = dot(phi, theta);
    const residual = measuredAccelerationWithoutDisturbance - prediction;

    if (regressorNormSq < minimumRegressorNorm * minimumRegressorNorm) {
      skippedUpdates += 1;
      lastDiagnostics = {
        updated: false,
        residual,
        regressorNorm: Math.sqrt(regressorNormSq),
        measuredAccelerationWithoutDisturbance,
        prediction,
        forgettingFactor: lambda,
      };
      return { parameters: getParameters(), covariance: getCovariance(), diagnostics: { ...lastDiagnostics } };
    }

    const Pphi = matVec(P, phi);
    const denominator = Math.max(1e-12, lambda + dot(phi, Pphi));
    const K = Pphi.map((value) => value / denominator);
    theta = theta.map((value, i) => value + K[i] * residual);
    theta = [
      clamp(theta[0], parameterBounds.stiffness[0], parameterBounds.stiffness[1]),
      clamp(theta[1], parameterBounds.damping[0], parameterBounds.damping[1]),
      clamp(theta[2], parameterBounds.gain[0], parameterBounds.gain[1]),
    ];

    const phiTP = [0, 1, 2].map((j) => phi[0] * P[0][j] + phi[1] * P[1][j] + phi[2] * P[2][j]);
    const nextP = P.map((row, i) => row.map((value, j) => (value - K[i] * phiTP[j]) / lambda));
    P = symmetrize(nextP);
    updates += 1;
    residualSqSum += residual * residual;

    lastDiagnostics = {
      updated: true,
      residual,
      residualRmse: Math.sqrt(residualSqSum / updates),
      regressorNorm: Math.sqrt(regressorNormSq),
      measuredAccelerationWithoutDisturbance,
      prediction,
      gainVector: [...K],
      covarianceTrace: P[0][0] + P[1][1] + P[2][2],
      forgettingFactor: lambda,
      updates,
      skippedUpdates,
    };

    return {
      parameters: getParameters(),
      covariance: getCovariance(),
      diagnostics: { ...lastDiagnostics, gainVector: [...K] },
    };
  }

  function getParameters() {
    return { stiffness: theta[0], damping: theta[1], gain: theta[2] };
  }

  function getCovariance() {
    return P.map((row) => [...row]);
  }

  function reset(parameters = initialParameters, covariance = initialCovariance) {
    theta = [
      finite(parameters.stiffness, 1),
      finite(parameters.damping, 1),
      finite(parameters.gain, 1),
    ];
    P = covariance3(covariance, 100);
    updates = 0;
    skippedUpdates = 0;
    residualSqSum = 0;
    lastDiagnostics = null;
  }

  return {
    update,
    getParameters,
    getCovariance,
    getDiagnostics() {
      return lastDiagnostics
        ? { ...lastDiagnostics, gainVector: lastDiagnostics.gainVector ? [...lastDiagnostics.gainVector] : undefined }
        : null;
    },
    reset,
  };
}
