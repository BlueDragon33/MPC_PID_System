import { createSeededGaussian } from './deterministicNoise.js';
import {
  linearizeKinematicBicycle,
  normalizeAngle,
  stepKinematicBicycle,
} from '../models/kinematicBicycle.js';

const zeros = (n, m) => Array.from({ length: n }, () => new Array(m).fill(0));
const identity = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));

function transpose(A) {
  return A[0].map((_, j) => A.map((row) => row[j]));
}

function matMul(A, B) {
  const out = zeros(A.length, B[0].length);
  for (let i = 0; i < A.length; i += 1) {
    for (let k = 0; k < B.length; k += 1) {
      const value = A[i][k];
      if (value === 0) continue;
      for (let j = 0; j < B[0].length; j += 1) out[i][j] += value * B[k][j];
    }
  }
  return out;
}

function matVec(A, x) {
  return A.map((row) => row.reduce((sum, value, j) => sum + value * x[j], 0));
}

function add(A, B) {
  return A.map((row, i) => row.map((value, j) => value + B[i][j]));
}

function subtract(A, B) {
  return A.map((row, i) => row.map((value, j) => value - B[i][j]));
}

function diagonal(values) {
  const out = zeros(values.length, values.length);
  values.forEach((value, i) => { out[i][i] = value; });
  return out;
}

function inverse(A) {
  const n = A.length;
  const aug = A.map((row, i) => [...row, ...identity(n)[i]]);
  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(aug[row][col]) > Math.abs(aug[pivot][col])) pivot = row;
    }
    if (Math.abs(aug[pivot][col]) < 1e-12) throw new Error('UGV EKF innovation covariance is singular.');
    [aug[col], aug[pivot]] = [aug[pivot], aug[col]];
    const scale = aug[col][col];
    aug[col] = aug[col].map((value) => value / scale);
    for (let row = 0; row < n; row += 1) {
      if (row === col) continue;
      const factor = aug[row][col];
      if (factor === 0) continue;
      aug[row] = aug[row].map((value, j) => value - factor * aug[col][j]);
    }
  }
  return aug.map((row) => row.slice(n));
}

function stateToVector(state) {
  return [state.x, state.y, state.yaw, state.v];
}

function vectorToState(vector) {
  return {
    x: vector[0],
    y: vector[1],
    yaw: normalizeAngle(vector[2]),
    v: vector[3],
  };
}

function covarianceTrace(P) {
  return P.reduce((sum, row, i) => sum + row[i], 0);
}

export function createUgvExtendedKalmanFilter(cfg, initialState) {
  const noise = createSeededGaussian(cfg.ugvEstimation.seed);
  const measurementStd = cfg.ugvEstimation.measurementStd;
  const Q = diagonal(cfg.ugvEstimation.processVariance);
  const R = diagonal(measurementStd.map((std) => std * std));
  let state = { ...initialState };
  let P = diagonal(cfg.ugvEstimation.initialVariance);

  function measure(truth) {
    return [
      truth.x + noise(0, measurementStd[0]),
      truth.y + noise(0, measurementStd[1]),
      normalizeAngle(truth.yaw + noise(0, measurementStd[2])),
      truth.v + noise(0, measurementStd[3]),
    ];
  }

  function updateWithMeasurement(predictedState, predictedCovariance, measurement) {
    const xPred = stateToVector(predictedState);
    const innovation = measurement.map((value, i) => value - xPred[i]);
    innovation[2] = normalizeAngle(innovation[2]);
    const S = add(predictedCovariance, R);
    const K = matMul(predictedCovariance, inverse(S));
    const correction = matVec(K, innovation);
    const updated = xPred.map((value, i) => value + correction[i]);
    updated[2] = normalizeAngle(updated[2]);

    const I = identity(4);
    const IK = subtract(I, K);
    // Joseph form, H = I.
    const updatedCovariance = add(
      matMul(matMul(IK, predictedCovariance), transpose(IK)),
      matMul(matMul(K, R), transpose(K)),
    );

    state = vectorToState(updated);
    P = updatedCovariance;
    return {
      state: { ...state },
      covariance: P.map((row) => [...row]),
      covarianceTrace: covarianceTrace(P),
      innovation,
      measurement,
    };
  }

  return {
    initialize(truth) {
      return updateWithMeasurement(state, P, measure(truth));
    },

    step(command, truth) {
      const A = linearizeKinematicBicycle(state, command, cfg).A;
      const predictedState = stepKinematicBicycle(state, command, cfg);
      const predictedCovariance = add(matMul(matMul(A, P), transpose(A)), Q);
      return updateWithMeasurement(predictedState, predictedCovariance, measure(truth));
    },

    getState() {
      return { ...state };
    },

    getCovariance() {
      return P.map((row) => [...row]);
    },
  };
}
