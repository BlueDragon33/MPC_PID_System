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
    A: rows.map((row) => denseRow(row, horizon)),
    b: rows.map((row) => row.bound),
    dimension: horizon,
    rateEnabled,
    deltaUMin: duMin,
    deltaUMax: duMax,
    uMin,
    uMax,
    previousU,
    form: 'A * U <= b',
  };
}

export function inequalityViolation(inequalities, x) {
  let maxViolation = 0;
  let violated = 0;
  for (const row of inequalities.rows) {
    const amount = sparseDot(row, x) - row.bound;
    if (amount > 0) {
      violated += 1;
      maxViolation = Math.max(maxViolation, amount);
    }
  }
  return { maxViolation, violated };
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

  const violation = inequalityViolation(inequalities, x);
  return {
    x,
    feasible: violation.maxViolation <= Math.max(tolerance, 1e-12),
    reason: violation.maxViolation <= Math.max(tolerance, 1e-12) ? 'ok' : 'repair-residual',
    maxViolation: violation.maxViolation,
    violated: violation.violated,
  };
}

export function projectPolyhedronDykstra(inequalities, input, options = {}) {
  const maxCycles = Math.max(1, Math.round(options.maxCycles ?? 12));
  const tolerance = Math.max(1e-14, options.tolerance ?? 1e-9);
  const rows = inequalities.rows;
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
    if (inequalityViolation(inequalities, x).maxViolation <= tolerance) break;
  }

  const preRepair = inequalityViolation(inequalities, x);
  const repaired = repairInputRateFeasibility(inequalities, x, tolerance);
  const feasibility = inequalityViolation(inequalities, repaired.x);

  return {
    x: repaired.x,
    cycles,
    dykstraConverged: preRepair.maxViolation <= tolerance,
    repairUsed: preRepair.maxViolation > tolerance,
    converged: repaired.feasible && feasibility.maxViolation <= tolerance,
    maxViolation: feasibility.maxViolation,
    violated: feasibility.violated,
  };
}
