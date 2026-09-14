import { buildInputRateInequalities } from './constraints.js';

const zeros = (rows, cols) => Array.from({ length: rows }, () => new Array(cols).fill(0));
const identity = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));

function transpose(A) {
  return A[0].map((_, j) => A.map((row) => row[j]));
}

function matMul(A, B) {
  const out = zeros(A.length, B[0].length);
  for (let i = 0; i < A.length; i += 1) {
    for (let k = 0; k < B.length; k += 1) {
      const aik = A[i][k];
      if (aik === 0) continue;
      for (let j = 0; j < B[0].length; j += 1) out[i][j] += aik * B[k][j];
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

function scale(A, scalar) {
  return A.map((row) => row.map((value) => value * scalar));
}

function addVectors(a, b) {
  return a.map((value, i) => value + b[i]);
}

function scaleVector(v, scalar) {
  return v.map((value) => value * scalar);
}

function dot(a, b) {
  return a.reduce((sum, value, i) => sum + value * b[i], 0);
}

function matrixPowers(A, maxPower) {
  const powers = [identity(A.length)];
  for (let k = 1; k <= maxPower; k += 1) powers.push(matMul(powers[k - 1], A));
  return powers;
}

function setBlock(target, row0, col0, block) {
  for (let i = 0; i < block.length; i += 1) {
    for (let j = 0; j < block[0].length; j += 1) target[row0 + i][col0 + j] = block[i][j];
  }
}

function vectorAsColumn(v) {
  return v.map((value) => [value]);
}

function blockDiagonalStateCost(N, cfg) {
  const n = 2;
  const Qbar = zeros(N * n, N * n);
  for (let k = 0; k < N; k += 1) {
    const weight = k === N - 1 ? cfg.mpc.terminalWeight : 1;
    Qbar[k * n][k * n] = weight * cfg.mpc.qPosition;
    Qbar[k * n + 1][k * n + 1] = weight * cfg.mpc.qVelocity;
  }
  return Qbar;
}

function diagonalInputCost(N, value) {
  const R = zeros(N, N);
  for (let i = 0; i < N; i += 1) R[i][i] = value;
  return R;
}

function deltaMatrix(N) {
  const D = zeros(N, N);
  D[0][0] = 1;
  for (let i = 1; i < N; i += 1) {
    D[i][i] = 1;
    D[i][i - 1] = -1;
  }
  return D;
}

export function buildPredictionMatrices(A, B, horizon) {
  const n = A.length;
  const m = 1;
  const Phi = zeros(horizon * n, n);
  const Gamma = zeros(horizon * n, horizon * m);
  const powers = matrixPowers(A, horizon);
  const Bcol = vectorAsColumn(B);

  for (let k = 1; k <= horizon; k += 1) {
    setBlock(Phi, (k - 1) * n, 0, powers[k]);
    for (let j = 0; j < k; j += 1) {
      const influence = matMul(powers[k - 1 - j], Bcol);
      setBlock(Gamma, (k - 1) * n, j, influence);
    }
  }

  return { Phi, Gamma };
}

export function buildCondensedQP({ A, B, state, target, previousU, cfg }) {
  const N = cfg.mpc.horizon;
  const { Phi, Gamma } = buildPredictionMatrices(A, B, N);
  const Qbar = blockDiagonalStateCost(N, cfg);
  const Rbar = diagonalInputCost(N, cfg.mpc.rInput);
  const D = deltaMatrix(N);
  const Dt = transpose(D);
  const GammaT = transpose(Gamma);

  const x0 = [state.x, state.v];
  const freePrediction = matVec(Phi, x0);
  const reference = Array.from({ length: N }, () => [target, 0]).flat();
  const stateOffset = freePrediction.map((value, i) => value - reference[i]);

  const stateHessian = matMul(matMul(GammaT, Qbar), Gamma);
  const deltaHessian = scale(matMul(Dt, D), cfg.mpc.rDelta);
  const baseHessian = add(add(stateHessian, Rbar), deltaHessian);
  const H = scale(baseHessian, 2);

  const stateLinear = matVec(matMul(GammaT, Qbar), stateOffset);
  const deltaReference = new Array(N).fill(0);
  deltaReference[0] = previousU;
  const deltaLinear = scaleVector(matVec(Dt, deltaReference), -cfg.mpc.rDelta);
  const f = scaleVector(addVectors(stateLinear, deltaLinear), 2);

  const lower = new Array(N).fill(cfg.mpc.uMin);
  const upper = new Array(N).fill(cfg.mpc.uMax);
  const inequalities = buildInputRateInequalities({
    horizon: N,
    uMin: cfg.mpc.uMin,
    uMax: cfg.mpc.uMax,
    deltaUMin: cfg.mpc.deltaUMin,
    deltaUMax: cfg.mpc.deltaUMax,
    previousU,
  });

  return {
    H,
    f,
    lower,
    upper,
    inequalities,
    Phi,
    Gamma,
    Qbar,
    Rbar,
    D,
    reference,
    freePrediction,
    form: '0.5 * U^T H U + f^T U, subject to A * U <= b',
  };
}

export function evaluateCondensedQP(qp, U) {
  const HU = matVec(qp.H, U);
  return 0.5 * dot(U, HU) + dot(qp.f, U);
}

export function qpDiagnostics(qp) {
  const n = qp.H.length;
  let maxSymmetryError = 0;
  let minDiagonal = Number.POSITIVE_INFINITY;
  let finite = true;

  for (let i = 0; i < n; i += 1) {
    minDiagonal = Math.min(minDiagonal, qp.H[i][i]);
    finite = finite && Number.isFinite(qp.f[i]);
    for (let j = 0; j < n; j += 1) {
      finite = finite && Number.isFinite(qp.H[i][j]);
      maxSymmetryError = Math.max(maxSymmetryError, Math.abs(qp.H[i][j] - qp.H[j][i]));
    }
  }

  const constraintFinite = qp.inequalities.rows.every((row) => Number.isFinite(row.bound) && row.values.every(Number.isFinite));
  return {
    dimension: n,
    inequalities: qp.inequalities.rows.length,
    rateConstraintsEnabled: qp.inequalities.rateEnabled,
    maxSymmetryError,
    minDiagonal,
    finite: finite && constraintFinite,
  };
}
