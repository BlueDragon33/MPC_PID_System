const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

function matVec2(A, x) {
  return [
    A[0][0] * x[0] + A[0][1] * x[1],
    A[1][0] * x[0] + A[1][1] * x[1],
  ];
}

function matMul2(A, B) {
  return [
    [
      A[0][0] * B[0][0] + A[0][1] * B[1][0],
      A[0][0] * B[0][1] + A[0][1] * B[1][1],
    ],
    [
      A[1][0] * B[0][0] + A[1][1] * B[1][0],
      A[1][0] * B[0][1] + A[1][1] * B[1][1],
    ],
  ];
}

function transpose2(A) {
  return [
    [A[0][0], A[1][0]],
    [A[0][1], A[1][1]],
  ];
}

function add2x2(A, B) {
  return [
    [A[0][0] + B[0][0], A[0][1] + B[0][1]],
    [A[1][0] + B[1][0], A[1][1] + B[1][1]],
  ];
}

function symmetrize(P) {
  const off = 0.5 * (P[0][1] + P[1][0]);
  return [
    [Math.max(0, finite(P[0][0])), finite(off)],
    [finite(off), Math.max(0, finite(P[1][1]))],
  ];
}

function covarianceFrom(value, fallbackDiagonal) {
  if (Array.isArray(value) && value.length === 2 && Array.isArray(value[0])) {
    return [
      [finite(value[0][0], fallbackDiagonal[0]), finite(value[0][1], 0)],
      [finite(value[1][0], 0), finite(value[1][1], fallbackDiagonal[1])],
    ];
  }
  if (Array.isArray(value) && value.length === 2) {
    return [[finite(value[0], fallbackDiagonal[0]), 0], [0, finite(value[1], fallbackDiagonal[1])]];
  }
  return [[fallbackDiagonal[0], 0], [0, fallbackDiagonal[1]]];
}

export function createLinearKalmanFilter({
  A,
  B,
  C = [1, 0],
  processCovariance = [1e-5, 1e-4],
  measurementVariance = 0.01,
  initialState = [0, 0],
  initialCovariance = [0.25, 0.5],
}) {
  if (!Array.isArray(A) || A.length !== 2 || !Array.isArray(A[0]) || A[0].length !== 2) {
    throw new Error('Kalman filter requires a 2x2 state-transition matrix A.');
  }
  if (!Array.isArray(B) || B.length !== 2) throw new Error('Kalman filter requires a two-element input vector B.');
  if (!Array.isArray(C) || C.length !== 2) throw new Error('Kalman filter requires a two-element measurement row C.');

  const Q = covarianceFrom(processCovariance, [1e-5, 1e-4]);
  const R = Math.max(1e-12, finite(measurementVariance, 0.01));
  let x = [finite(initialState[0]), finite(initialState[1])];
  let P = covarianceFrom(initialCovariance, [0.25, 0.5]);
  let lastDiagnostics = null;

  function predict(u = 0) {
    const Ax = matVec2(A, x);
    x = [Ax[0] + B[0] * u, Ax[1] + B[1] * u];
    P = symmetrize(add2x2(matMul2(matMul2(A, P), transpose2(A)), Q));
    return { x: x[0], v: x[1], covariance: P.map((row) => [...row]) };
  }

  function update(measurement) {
    const y = finite(measurement);
    const predictedMeasurement = C[0] * x[0] + C[1] * x[1];
    const innovation = y - predictedMeasurement;
    const PCt = [
      P[0][0] * C[0] + P[0][1] * C[1],
      P[1][0] * C[0] + P[1][1] * C[1],
    ];
    const innovationVariance = Math.max(1e-12, C[0] * PCt[0] + C[1] * PCt[1] + R);
    const K = [PCt[0] / innovationVariance, PCt[1] / innovationVariance];

    x = [x[0] + K[0] * innovation, x[1] + K[1] * innovation];

    // Joseph-form covariance update preserves symmetry/positive semidefiniteness
    // better than the compact (I-KC)P form under finite precision.
    const M = [
      [1 - K[0] * C[0], -K[0] * C[1]],
      [-K[1] * C[0], 1 - K[1] * C[1]],
    ];
    const KRKt = [
      [K[0] * R * K[0], K[0] * R * K[1]],
      [K[1] * R * K[0], K[1] * R * K[1]],
    ];
    P = symmetrize(add2x2(matMul2(matMul2(M, P), transpose2(M)), KRKt));

    lastDiagnostics = {
      measurement: y,
      predictedMeasurement,
      innovation,
      innovationVariance,
      kalmanGain: [...K],
      covarianceTrace: P[0][0] + P[1][1],
    };

    return {
      x: x[0],
      v: x[1],
      covariance: P.map((row) => [...row]),
      diagnostics: { ...lastDiagnostics, kalmanGain: [...K] },
    };
  }

  function step(u, measurement) {
    predict(u);
    return update(measurement);
  }

  return {
    predict,
    update,
    step,
    getState() {
      return { x: x[0], v: x[1] };
    },
    getCovariance() {
      return P.map((row) => [...row]);
    },
    getDiagnostics() {
      return lastDiagnostics ? { ...lastDiagnostics, kalmanGain: [...lastDiagnostics.kalmanGain] } : null;
    },
    reset(state = initialState, covariance = initialCovariance) {
      x = [finite(state[0]), finite(state[1])];
      P = covarianceFrom(covariance, [0.25, 0.5]);
      lastDiagnostics = null;
    },
  };
}
