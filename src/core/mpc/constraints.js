const finiteOr = (value, fallback) => Number.isFinite(value) ? value : fallback;
const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

function sparseDot(row, x) {
  let sum = 0;
  for (let k = 0; k < row.indices.length; k += 1) sum += row.values[k] * x[row.indices[k]];
  return sum;
}

function rowNormSquared(row) {
  return row.values.reduce((sum, value) => sum + value * value, 0);
}

function denseRow(row, dimension) {
  const out = new Array(dimension).fill(0);
  for (let k = 0; k < row.indices.length; k += 1) out[row.indices[k]] = row.values[k];
  return out;
}

function sparseFromDense(values, epsilon = 1e-14) {
  const indices = [];
  const sparseValues = [];
  for (let i = 0; i < values.length; i += 1) {
    if (Math.abs(values[i]) > epsilon) {
      indices.push(i);
      sparseValues.push(values[i]);
    }
  }
  return { indices, values: sparseValues };
}

function addDenseConstraint(rows, coefficients, bound, kind, stage, meta = {}) {
  if (!Number.isFinite(bound)) return;
  const sparse = sparseFromDense(coefficients);
  rows.push({ ...sparse, bound, kind, stage, ...meta });
}

export function buildInputRateInequalities({ horizon, uMin, uMax, deltaUMin, deltaUMax, previousU }) {
  const rows = [];
  const add = (indices, values, bound, kind, stage) => rows.push({ indices, values, bound, kind, stage });

  for (let i = 0; i < horizon; i += 1) {
    add([i], [1], uMax, 'input-upper', i);
    add([i], [-1], -uMin, 'input-lower', i);
  }

  const duMin = finiteOr(deltaUMin, Number.NEGATIVE_INFINITY);
  const duMax = finiteOr(deltaUMax, Number.POSITIVE_INFINITY);
  const rateEnabled = Number.isFinite(duMin) || Number.isFinite(duMax);

  if (rateEnabled) {
    for (let i = 0; i < horizon; i += 1) {
      const indices = i === 0 ? [0] : [i - 1, i];
      const forward = i === 0 ? [1] : [-1, 1];
      const reverse = forward.map((value) => -value);
      const offset = i === 0 ? previousU : 0;

      if (Number.isFinite(duMax)) add(indices, forward, duMax + offset, 'rate-upper', i);
      if (Number.isFinite(duMin)) add(indices, reverse, -duMin - offset, 'rate-lower', i);
    }
  }

  return {
    rows,
    dimension: horizon,
    rateEnabled,
    deltaUMin: duMin,
    deltaUMax: duMax,
    uMin,
    uMax,
    previousU,
    stateConstraintsEnabled: false,
    outputConstraintsEnabled: false,
  };
}

export function appendPredictionInequalities(inequalities, {
  Gamma,
  freePrediction,
  horizon,
  stateDimension = 2,
  outputC = [1, 0],
  positionMin,
  positionMax,
  velocityMin,
  velocityMax,
  outputMin,
  outputMax,
}) {
  const rows = inequalities.rows;
  let stateCount = 0;
  let outputCount = 0;

  for (let k = 0; k < horizon; k += 1) {
    const xRowIndex = k * stateDimension;
    const vRowIndex = xRowIndex + 1;
    const gx = Gamma[xRowIndex];
    const gv = Gamma[vRowIndex];
    const freeX = freePrediction[xRowIndex];
    const freeV = freePrediction[vRowIndex];

    if (Number.isFinite(positionMax)) {
      addDenseConstraint(rows, gx, positionMax - freeX, 'state-position-upper', k, { variable: 'position' });
      stateCount += 1;
    }
    if (Number.isFinite(positionMin)) {
      addDenseConstraint(rows, gx.map((value) => -value), freeX - positionMin, 'state-position-lower', k, { variable: 'position' });
      stateCount += 1;
    }
    if (Number.isFinite(velocityMax)) {
      addDenseConstraint(rows, gv, velocityMax - freeV, 'state-velocity-upper', k, { variable: 'velocity' });
      stateCount += 1;
    }
    if (Number.isFinite(velocityMin)) {
      addDenseConstraint(rows, gv.map((value) => -value), freeV - velocityMin, 'state-velocity-lower', k, { variable: 'velocity' });
      stateCount += 1;
    }

    const outputGamma = gx.map((value, j) => (outputC[0] ?? 0) * value + (outputC[1] ?? 0) * gv[j]);
    const freeOutput = (outputC[0] ?? 0) * freeX + (outputC[1] ?? 0) * freeV;
    if (Number.isFinite(outputMax)) {
      addDenseConstraint(rows, outputGamma, outputMax - freeOutput, 'output-upper', k, { variable: 'output' });
      outputCount += 1;
    }
    if (Number.isFinite(outputMin)) {
      addDenseConstraint(rows, outputGamma.map((value) => -value), freeOutput - outputMin, 'output-lower', k, { variable: 'output' });
      outputCount += 1;
    }
  }

  inequalities.stateConstraintsEnabled = stateCount > 0;
  inequalities.outputConstraintsEnabled = outputCount > 0;
  inequalities.stateConstraintCount = stateCount;
  inequalities.outputConstraintCount = outputCount;
  inequalities.A = rows.map((row) => denseRow(row, inequalities.dimension));
  inequalities.b = rows.map((row) => row.bound);
  inequalities.form = 'A * U <= b';
  return inequalities;
}

export function finalizeInequalities(inequalities) {
  inequalities.A = inequalities.rows.map((row) => denseRow(row, inequalities.dimension));
  inequalities.b = inequalities.rows.map((row) => row.bound);
  inequalities.form = 'A * U <= b';
  return inequalities;
}

export function inequalityViolation(inequalities, x) {
  let maxViolation = 0;
  let violated = 0;
  let worst = null;
  for (const row of inequalities.rows) {
    const amount = sparseDot(row, x) - row.bound;
    if (amount > 0) {
      violated += 1;
      if (amount > maxViolation) {
        maxViolation = amount;
        worst = { kind: row.kind, stage: row.stage, amount };
      }
    }
  }
  return { maxViolation, violated, worst };
}

export function activeInequalityStats(inequalities, x, tolerance = 1e-6) {
  const byKind = {};
  let total = 0;
  for (const row of inequalities.rows) {
    const slack = row.bound - sparseDot(row, x);
    if (Math.abs(slack) <= tolerance) {
      total += 1;
      byKind[row.kind] = (byKind[row.kind] || 0) + 1;
    }
  }
  return {
    total,
    ratio: inequalities.rows.length ? total / inequalities.rows.length : 0,
    byKind,
  };
}

export function precheckInputRateFeasibility(inequalities, previousU, uMin, uMax, tolerance = 1e-12) {
  if (!(Number.isFinite(uMin) && Number.isFinite(uMax)) || uMin > uMax) {
    return { feasible: false, reason: 'invalid-input-bounds' };
  }
  if (inequalities.deltaUMin > inequalities.deltaUMax) {
    return { feasible: false, reason: 'invalid-rate-bounds' };
  }

  if (inequalities.rateEnabled) {
    const firstLower = Math.max(uMin, previousU + inequalities.deltaUMin);
    const firstUpper = Math.min(uMax, previousU + inequalities.deltaUMax);
    if (firstLower > firstUpper + tolerance) {
      return { feasible: false, reason: 'first-move-rate-conflicts-with-input-bounds', firstLower, firstUpper };
    }
  }

  return { feasible: true, reason: 'ok' };
}

export function repairInputRateFeasibility(inequalities, input, tolerance = 1e-12) {
  const x = [...input];
  let previous = inequalities.previousU;

  for (let i = 0; i < x.length; i += 1) {
    let lower = inequalities.uMin;
    let upper = inequalities.uMax;

    if (inequalities.rateEnabled) {
      if (Number.isFinite(inequalities.deltaUMin)) lower = Math.max(lower, previous + inequalities.deltaUMin);
      if (Number.isFinite(inequalities.deltaUMax)) upper = Math.min(upper, previous + inequalities.deltaUMax);
    }

    if (lower > upper + tolerance) {
      return { x, feasible: false, reason: 'rate-input-intersection-empty', stage: i, lower, upper };
    }

    x[i] = clamp(x[i], lower, upper);
    previous = x[i];
  }

  const inputRateRows = { ...inequalities, rows: inequalities.rows.filter((row) => row.kind.startsWith('input-') || row.kind.startsWith('rate-')) };
  const violation = inequalityViolation(inputRateRows, x);
  return {
    x,
    feasible: violation.maxViolation <= Math.max(tolerance, 1e-12),
    reason: violation.maxViolation <= Math.max(tolerance, 1e-12) ? 'ok' : 'repair-residual',
    maxViolation: violation.maxViolation,
    violated: violation.violated,
  };
}

function runDykstra(rows, input, maxCycles, tolerance) {
  const x = [...input];
  const corrections = rows.map((row) => new Array(row.indices.length).fill(0));
  let cycles = 0;

  for (let cycle = 1; cycle <= maxCycles; cycle += 1) {
    for (let r = 0; r < rows.length; r += 1) {
      const row = rows[r];
      const correction = corrections[r];
      const yValues = new Array(row.indices.length);
      let dot = 0;

      for (let k = 0; k < row.indices.length; k += 1) {
        const y = x[row.indices[k]] + correction[k];
        yValues[k] = y;
        dot += row.values[k] * y;
      }

      const excess = dot - row.bound;
      const norm2 = rowNormSquared(row);
      const scale = excess > 0 && norm2 > 0 ? excess / norm2 : 0;

      for (let k = 0; k < row.indices.length; k += 1) {
        const next = yValues[k] - scale * row.values[k];
        correction[k] = yValues[k] - next;
        x[row.indices[k]] = next;
      }
    }

    cycles = cycle;
    const current = { rows };
    if (inequalityViolation(current, x).maxViolation <= tolerance) break;
  }

  return { x, cycles };
}

export function projectPolyhedronDykstra(inequalities, input, options = {}) {
  const maxCycles = Math.max(1, Math.round(options.maxCycles ?? 12));
  const tolerance = Math.max(1e-14, options.tolerance ?? 1e-9);
  const generalConstraints = inequalities.stateConstraintsEnabled || inequalities.outputConstraintsEnabled;

  let projection = runDykstra(inequalities.rows, input, maxCycles, tolerance);
  let x = projection.x;
  let cycles = projection.cycles;
  let preRepair = inequalityViolation(inequalities, x);
  let repairUsed = false;

  if (!generalConstraints && preRepair.maxViolation > tolerance) {
    const repaired = repairInputRateFeasibility(inequalities, x, tolerance);
    x = repaired.x;
    repairUsed = true;
  } else if (generalConstraints && preRepair.maxViolation > tolerance) {
    // General state/output rows need the true polyhedral projection. Give Dykstra
    // an adaptive refinement budget instead of applying a rate-only repair that
    // could silently violate a predicted safety envelope.
    const refinement = runDykstra(inequalities.rows, x, Math.max(maxCycles * 4, 24), tolerance);
    x = refinement.x;
    cycles += refinement.cycles;
  }

  const feasibility = inequalityViolation(inequalities, x);
  return {
    x,
    cycles,
    dykstraConverged: preRepair.maxViolation <= tolerance,
    repairUsed,
    converged: feasibility.maxViolation <= tolerance,
    maxViolation: feasibility.maxViolation,
    violated: feasibility.violated,
    worstViolation: feasibility.worst,
  };
}
