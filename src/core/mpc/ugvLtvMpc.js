import { finalizeInequalities } from './constraints.js';
import { solveGenericConstrainedQP } from '../solvers/genericConstrainedQP.js';
import { ugvReferenceAtX, ugvTrackingErrors } from '../models/ugvReferencePath.js';

const zeros = (rows, cols) => Array.from({ length: rows }, () => new Array(cols).fill(0));
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

function scale(A, factor) {
  return A.map((row) => row.map((value) => value * factor));
}

function matrixPowers(A, horizon) {
  const powers = [identity(A.length)];
  for (let i = 1; i <= horizon; i += 1) powers.push(matMul(powers[i - 1], A));
  return powers;
}

function setBlock(target, row0, col0, block) {
  for (let i = 0; i < block.length; i += 1) {
    for (let j = 0; j < block[0].length; j += 1) target[row0 + i][col0 + j] = block[i][j];
  }
}

function predictionMatrices(A, B, horizon) {
  const n = A.length;
  const m = B[0].length;
  const Phi = zeros(horizon * n, n);
  const Gamma = zeros(horizon * n, horizon * m);
  const powers = matrixPowers(A, horizon);

  for (let k = 1; k <= horizon; k += 1) {
    setBlock(Phi, (k - 1) * n, 0, powers[k]);
    for (let j = 0; j < k; j += 1) {
      setBlock(Gamma, (k - 1) * n, j * m, matMul(powers[k - 1 - j], B));
    }
  }
  return { Phi, Gamma };
}

function diagonalCost(horizon, stageValues, terminalScale = 1) {
  const n = stageValues.length;
  const out = zeros(horizon * n, horizon * n);
  for (let k = 0; k < horizon; k += 1) {
    const scaleFactor = k === horizon - 1 ? terminalScale : 1;
    for (let i = 0; i < n; i += 1) out[k * n + i][k * n + i] = stageValues[i] * scaleFactor;
  }
  return out;
}

function deltaMatrix(horizon, inputs) {
  const dimension = horizon * inputs;
  const D = zeros(dimension, dimension);
  for (let k = 0; k < horizon; k += 1) {
    for (let j = 0; j < inputs; j += 1) {
      const row = k * inputs + j;
      D[row][row] = 1;
      if (k > 0) D[row][row - inputs] = -1;
    }
  }
  return D;
}

function sparseRow(coefficients, bound, kind, stage, variable) {
  const indices = [];
  const values = [];
  coefficients.forEach((value, index) => {
    if (Math.abs(value) > 1e-14) {
      indices.push(index);
      values.push(value);
    }
  });
  return { indices, values, bound, kind, stage, variable };
}

function addBound(rows, dimension, index, coefficient, bound, kind, stage, variable) {
  const row = new Array(dimension).fill(0);
  row[index] = coefficient;
  rows.push(sparseRow(row, bound, kind, stage, variable));
}

function buildErrorModel(reference, cfg) {
  const dt = cfg.ugvLtvMpc.predictionDt ?? cfg.ugvLtvMpc.solveInterval ?? cfg.dt;
  const speed = Math.max(0.2, reference.speed);
  const wheelbase = cfg.ugv.wheelbase;
  const steeringFeedforward = Math.atan(wheelbase * reference.curvature);
  const steeringGain = dt * speed / wheelbase / (Math.cos(steeringFeedforward) ** 2);

  return {
    A: [
      [1, dt * speed, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
    B: [
      [0, 0],
      [0, steeringGain],
      [dt, 0],
    ],
    steeringFeedforward,
  };
}

export function buildUgvLtvQp(state, previousCommand, cfg) {
  const reference = ugvReferenceAtX(state.x, cfg);
  const error = ugvTrackingErrors(state, reference);
  const z0 = [error.lateral, error.heading, error.speed];
  const { A, B, steeringFeedforward } = buildErrorModel(reference, cfg);
  const horizon = cfg.ugvLtvMpc.horizon;
  const inputs = 2;
  const { Phi, Gamma } = predictionMatrices(A, B, horizon);
  const freePrediction = matVec(Phi, z0);

  const Q = diagonalCost(
    horizon,
    [cfg.ugvLtvMpc.qLateral, cfg.ugvLtvMpc.qHeading, cfg.ugvLtvMpc.qSpeed],
    cfg.ugvLtvMpc.terminalScale,
  );
  const R = diagonalCost(
    horizon,
    [cfg.ugvLtvMpc.rAcceleration, cfg.ugvLtvMpc.rSteering],
    1,
  );
  const Rd = diagonalCost(
    horizon,
    [cfg.ugvLtvMpc.rDeltaAcceleration, cfg.ugvLtvMpc.rDeltaSteering],
    1,
  );
  const D = deltaMatrix(horizon, inputs);
  const Gt = transpose(Gamma);
  const Dt = transpose(D);
  const H = scale(add(add(matMul(matMul(Gt, Q), Gamma), R), matMul(matMul(Dt, Rd), D)), 2);

  const stateLinear = matVec(matMul(Gt, Q), freePrediction);
  const previousDeviation = [
    previousCommand?.acceleration ?? 0,
    (previousCommand?.steering ?? 0) - steeringFeedforward,
  ];
  const deltaReference = new Array(horizon * inputs).fill(0);
  deltaReference[0] = previousDeviation[0];
  deltaReference[1] = previousDeviation[1];
  const deltaLinear = matVec(matMul(Dt, Rd), deltaReference).map((value) => -value);
  const f = stateLinear.map((value, i) => 2 * (value + deltaLinear[i]));

  const dimension = horizon * inputs;
  const rows = [];
  const accelMin = cfg.ugv.accelerationMin;
  const accelMax = cfg.ugv.accelerationMax;
  const steerErrorMin = cfg.ugv.steeringMin - steeringFeedforward;
  const steerErrorMax = cfg.ugv.steeringMax - steeringFeedforward;
  const predictionDt = cfg.ugvLtvMpc.predictionDt ?? cfg.ugvLtvMpc.solveInterval ?? cfg.dt;
  const accelRateMin = cfg.ugv.accelerationRateMin * predictionDt;
  const accelRateMax = cfg.ugv.accelerationRateMax * predictionDt;
  const steerRateMin = cfg.ugv.steeringRateMin * predictionDt;
  const steerRateMax = cfg.ugv.steeringRateMax * predictionDt;

  for (let k = 0; k < horizon; k += 1) {
    const aIndex = k * inputs;
    const sIndex = aIndex + 1;
    addBound(rows, dimension, aIndex, 1, accelMax, 'input-accel-upper', k, 'acceleration');
    addBound(rows, dimension, aIndex, -1, -accelMin, 'input-accel-lower', k, 'acceleration');
    addBound(rows, dimension, sIndex, 1, steerErrorMax, 'input-steer-upper', k, 'steering');
    addBound(rows, dimension, sIndex, -1, -steerErrorMin, 'input-steer-lower', k, 'steering');

    const aRate = new Array(dimension).fill(0);
    const sRate = new Array(dimension).fill(0);
    aRate[aIndex] = 1;
    sRate[sIndex] = 1;
    if (k > 0) {
      aRate[aIndex - inputs] = -1;
      sRate[sIndex - inputs] = -1;
    }
    const aOffset = k === 0 ? previousDeviation[0] : 0;
    const sOffset = k === 0 ? previousDeviation[1] : 0;
    rows.push(sparseRow(aRate, accelRateMax + aOffset, 'rate-accel-upper', k, 'acceleration'));
    rows.push(sparseRow(aRate.map((value) => -value), -accelRateMin - aOffset, 'rate-accel-lower', k, 'acceleration'));
    rows.push(sparseRow(sRate, steerRateMax + sOffset, 'rate-steer-upper', k, 'steering'));
    rows.push(sparseRow(sRate.map((value) => -value), -steerRateMin - sOffset, 'rate-steer-lower', k, 'steering'));
  }

  for (let k = 0; k < horizon; k += 1) {
    const row0 = k * 3;
    const lateralGamma = Gamma[row0];
    const headingGamma = Gamma[row0 + 1];
    const speedGamma = Gamma[row0 + 2];
    const freeLateral = freePrediction[row0];
    const freeHeading = freePrediction[row0 + 1];
    const freeSpeed = freePrediction[row0 + 2];

    rows.push(sparseRow(
      lateralGamma,
      cfg.ugvSafety.laneHalfWidth - freeLateral,
      'state-lateral-upper',
      k,
      'lateral',
    ));
    rows.push(sparseRow(
      lateralGamma.map((value) => -value),
      cfg.ugvSafety.laneHalfWidth + freeLateral,
      'state-lateral-lower',
      k,
      'lateral',
    ));

    const headingLimit = cfg.ugvSafety.headingAbsMax ?? 0.9;
    rows.push(sparseRow(
      headingGamma,
      headingLimit - freeHeading,
      'state-heading-upper',
      k,
      'heading',
    ));
    rows.push(sparseRow(
      headingGamma.map((value) => -value),
      headingLimit + freeHeading,
      'state-heading-lower',
      k,
      'heading',
    ));

    const speedErrorMin = cfg.ugvSafety.speedMin - reference.speed;
    const speedErrorMax = cfg.ugvSafety.speedMax - reference.speed;
    rows.push(sparseRow(
      speedGamma,
      speedErrorMax - freeSpeed,
      'state-speed-upper',
      k,
      'speed',
    ));
    rows.push(sparseRow(
      speedGamma.map((value) => -value),
      freeSpeed - speedErrorMin,
      'state-speed-lower',
      k,
      'speed',
    ));
  }

  const inequalities = finalizeInequalities({
    rows,
    dimension,
    rateEnabled: true,
    stateConstraintsEnabled: true,
    outputConstraintsEnabled: false,
  });

  return {
    H,
    f,
    inequalities,
    Phi,
    Gamma,
    freePrediction,
    reference,
    error,
    steeringFeedforward,
    form: '0.5 * U^T H U + f^T U, U=[a,delta_error] over horizon',
  };
}

function shiftWarmStart(sequence, horizon, stages = 1) {
  if (!sequence || sequence.length !== horizon * 2) return null;
  const shift = Math.max(1, Math.min(horizon, Math.round(stages))) * 2;
  const tail = sequence.slice(-2);
  const shifted = sequence.slice(shift);
  while (shifted.length < horizon * 2) shifted.push(...tail);
  return shifted.slice(0, horizon * 2);
}

export function solveUgvLtvMpc(
  state,
  previousCommand,
  cfg,
  warmStart = null,
  warmStartShiftStages = 1,
) {
  const qp = buildUgvLtvQp(state, previousCommand, cfg);
  const horizon = cfg.ugvLtvMpc.horizon;
  const previousDeviation = [
    previousCommand?.acceleration ?? 0,
    (previousCommand?.steering ?? 0) - qp.steeringFeedforward,
  ];
  const holdCurrentCommandSeed = Array.from(
    { length: horizon },
    () => previousDeviation,
  ).flat();

  const solution = solveGenericConstrainedQP(qp, {
    maxIterations: cfg.ugvLtvMpc.qpIterations,
    tolerance: cfg.ugvLtvMpc.qpTolerance,
    feasibilityTolerance: cfg.ugvLtvMpc.qpFeasibilityTolerance,
    projectionCycles: cfg.ugvLtvMpc.qpProjectionCycles,
    projectionTolerance: cfg.ugvLtvMpc.qpProjectionTolerance,
    stepScale: cfg.ugvLtvMpc.qpStepScale,
    feasibleSeed: holdCurrentCommandSeed,
  }, shiftWarmStart(warmStart, horizon, warmStartShiftStages));

  const accepted = solution.status === 'solved' || solution.status === 'max-iterations';
  const acceleration = accepted ? solution.x[0] : previousCommand.acceleration;
  const steering = accepted
    ? qp.steeringFeedforward + solution.x[1]
    : previousCommand.steering;
  const command = {
    acceleration: Math.max(cfg.ugv.accelerationMin, Math.min(cfg.ugv.accelerationMax, acceleration)),
    steering: Math.max(cfg.ugv.steeringMin, Math.min(cfg.ugv.steeringMax, steering)),
  };

  return {
    command,
    sequence: solution.x,
    status: solution.status,
    solveMs: solution.solveMs,
    objective: solution.objective,
    diagnostics: solution.diagnostics,
    fallbackUsed: !accepted,
    reference: qp.reference,
    error: qp.error,
    steeringFeedforward: qp.steeringFeedforward,
    qp,
  };
}
