const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

function zeros3() {
  return [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
}

function matVec3(A, x) {
  return A.map((row) => row[0] * x[0] + row[1] * x[1] + row[2] * x[2]);
}

function matMul3(A, B) {
  const out = zeros3();
  for (let i = 0; i < 3; i += 1) {
    for (let j = 0; j < 3; j += 1) {
      for (let k = 0; k < 3; k += 1) out[i][j] += A[i][k] * B[k][j];
    }
  }
  return out;
}

function transpose3(A) {
  return [
    [A[0][0], A[1][0], A[2][0]],
    [A[0][1], A[1][1], A[2][1]],
    [A[0][2], A[1][2], A[2][2]],
  ];
}

function add3(A, B) {
  return A.map((row, i) => row.map((value, j) => value + B[i][j]));
}

function symmetrize3(P) {
  const out = zeros3();
  for (let i = 0; i < 3; i += 1) {
    for (let j = 0; j < 3; j += 1) {
      const value = 0.5 * (finite(P[i][j]) + finite(P[j][i]));
      out[i][j] = i === j ? Math.max(0, value) : value;
    }
  }
  return out;
}

function covariance3(value, fallback) {
  if (Array.isArray(value) && value.length === 3 && value.every(Array.isArray)) {
    return value.map((row, i) => row.map((entry, j) => finite(entry, i === j ? fallback[i] : 0)));
  }
  if (Array.isArray(value) && value.length === 3) {
    return [[finite(value[0], fallback[0]), 0, 0], [0, finite(value[1], fallback[1]), 0], [0, 0, finite(value[2], fallback[2])]];
  }
  return [[fallback[0], 0, 0], [0, fallback[1], 0], [0, 0, fallback[2]]];
}

export function createAugmentedDisturbanceKalmanFilter({
  A,
  B,
  E = [0, 0],
  C = [1, 0],
  processCovariance = [1e-5, 1e-4, 1e-3],
  measurementVariance = 0.01,
  initialState = [0, 0, 0],
  initialCovariance = [0.25, 0.5, 1.0],
  disturbanceRetention = 1,
}) {
  if (!Array.isArray(A) || A.length !== 2 || !Array.isArray(A[0]) || A[0].length !== 2) {
    throw new Error('Augmented Kalman filter requires a 2x2 plant matrix A.');
  }
  if (!Array.isArray(B) || B.length !== 2) throw new Error('Augmented Kalman filter requires a two-element B.');
  if (!Array.isArray(E) || E.length !== 2) throw new Error('Augmented Kalman filter requires a two-element disturbance vector E.');
  if (!Array.isArray(C) || C.length !== 2) throw new Error('Augmented Kalman filter requires a two-element measurement row C.');

  const rhoD = Math.max(0, Math.min(1, finite(disturbanceRetention, 1)));
  const Aaug = [
    [A[0][0], A[0][1], E[0]],
    [A[1][0], A[1][1], E[1]],
    [0, 0, rhoD],
  ];
  const Baug = [B[0], B[1], 0];
  const Caug = [C[0], C[1], 0];
  const Q = covariance3(processCovariance, [1e-5, 1e-4, 1e-3]);
  const R = Math.max(1e-12, finite(measurementVariance, 0.01));
  let x = [finite(initialState[0]), finite(initialState[1]), finite(initialState[2])];
  let P = covariance3(initialCovariance, [0.25, 0.5, 1.0]);
  let lastDiagnostics = null;

  function predict(u = 0) {
    const Ax = matVec3(Aaug, x);
    x = Ax.map((value, i) => value + Baug[i] * u);
    P = symmetrize3(add3(matMul3(matMul3(Aaug, P), transpose3(Aaug)), Q));
    return { x: x[0], v: x[1], d: x[2], covariance: P.map((row) => [...row]) };
  }

  function update(measurement) {
    const y = finite(measurement);
    const predictedMeasurement = Caug[0] * x[0] + Caug[1] * x[1] + Caug[2] * x[2];
    const innovation = y - predictedMeasurement;
    const PCt = P.map((row) => row[0] * Caug[0] + row[1] * Caug[1] + row[2] * Caug[2]);
    const innovationVariance = Math.max(1e-12, Caug[0] * PCt[0] + Caug[1] * PCt[1] + Caug[2] * PCt[2] + R);
    const K = PCt.map((value) => value / innovationVariance);
    x = x.map((value, i) => value + K[i] * innovation);

    const M = [
      [1 - K[0] * Caug[0], -K[0] * Caug[1], -K[0] * Caug[2]],
      [-K[1] * Caug[0], 1 - K[1] * Caug[1], -K[1] * Caug[2]],
      [-K[2] * Caug[0], -K[2] * Caug[1], 1 - K[2] * Caug[2]],
    ];
    const KRKt = K.map((ki) => K.map((kj) => ki * R * kj));
    P = symmetrize3(add3(matMul3(matMul3(M, P), transpose3(M)), KRKt));

    lastDiagnostics = {
      measurement: y,
      predictedMeasurement,
      innovation,
      innovationVariance,
      kalmanGain: [...K],
      covarianceTrace: P[0][0] + P[1][1] + P[2][2],
      disturbanceVariance: P[2][2],
      disturbanceRetention: rhoD,
    };

    return {
      x: x[0],
      v: x[1],
      d: x[2],
      covariance: P.map((row) => [...row]),
      diagnostics: { ...lastDiagnostics, kalmanGain: [...K] },
    };
  }

  function step(u, measurement) {
    predict(u);
    return update(measurement);
  }

  return {
    Aaug,
    Baug,
    Caug,
    disturbanceRetention: rhoD,
    predict,
    update,
    step,
    getState() {
      return { x: x[0], v: x[1], d: x[2] };
    },
    getCovariance() {
      return P.map((row) => [...row]);
    },
    getDiagnostics() {
      return lastDiagnostics ? { ...lastDiagnostics, kalmanGain: [...lastDiagnostics.kalmanGain] } : null;
    },
    reset(state = initialState, covariance = initialCovariance) {
      x = [finite(state[0]), finite(state[1]), finite(state[2])];
      P = covariance3(covariance, [0.25, 0.5, 1.0]);
      lastDiagnostics = null;
    },
  };
}
