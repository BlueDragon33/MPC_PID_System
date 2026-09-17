import { defaultConfig, runSimulation } from '../simulator.js';
import { applyExperimentPreset, getExperimentPreset } from './presets.js';

const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function applyMatrixOverrides(config, { seed, noiseScale, mismatchScale }) {
  const cfg = clone(config);
  if (Number.isFinite(seed)) cfg.estimation.seed = Math.round(seed);
  if (Number.isFinite(noiseScale)) {
    cfg.estimation.measurementNoiseStd = Math.max(0, finite(cfg.estimation.measurementNoiseStd) * noiseScale);
  }
  if (cfg.truthPlant.enabled && Number.isFinite(mismatchScale)) {
    for (const key of ['stiffnessScale', 'dampingScale', 'gainScale']) {
      const base = finite(cfg.truthPlant[key], 1);
      cfg.truthPlant[key] = 1 + (base - 1) * mismatchScale;
    }
  }
  return cfg;
}

function mean(values) {
  const finiteValues = values.filter(Number.isFinite);
  return finiteValues.length ? finiteValues.reduce((sum, value) => sum + value, 0) / finiteValues.length : null;
}

function compactTrace(samples) {
  return samples.map((sample) => ({
    t: sample.t,
    x: sample.x,
    v: sample.v,
    controllerX: sample.controllerX,
    controllerV: sample.controllerV,
    measurement: sample.measurement,
    estimateX: sample.estimateX,
    u: sample.u,
    reference: sample.reference,
    disturbance: sample.disturbance,
    covarianceTrace: sample.covarianceTrace,
    triggered: Boolean(sample.triggered),
    triggerReason: sample.triggerReason || '',
    solverStatus: sample.solverStatus || null,
    fallbackUsed: Boolean(sample.fallbackUsed),
    safetyViolation: finite(sample.safetyViolation),
    governorSafetyIntervened: Boolean(sample.governorSafetyIntervened),
  }));
}

function compactSolverRecords(records) {
  return records.map((record) => ({
    solveMs: record.solveMs,
    solver: record.solver,
    status: record.status,
    fallbackUsed: Boolean(record.fallbackUsed),
    fallbackReason: record.fallbackReason || null,
  }));
}

export function compareExperimentCases(a, b) {
  const aUnsafe = finite(a?.metrics?.safetyViolationCount);
  const bUnsafe = finite(b?.metrics?.safetyViolationCount);
  if (aUnsafe !== bUnsafe) return aUnsafe - bUnsafe;
  const aSafety = finite(a?.metrics?.maxActualSafetyViolation);
  const bSafety = finite(b?.metrics?.maxActualSafetyViolation);
  if (aSafety !== bSafety) return aSafety - bSafety;
  const aFallback = finite(a?.metrics?.fallbackCount);
  const bFallback = finite(b?.metrics?.fallbackCount);
  if (aFallback !== bFallback) return aFallback - bFallback;
  const aIae = finite(a?.metrics?.iae, Number.POSITIVE_INFINITY);
  const bIae = finite(b?.metrics?.iae, Number.POSITIVE_INFINITY);
  if (aIae !== bIae) return aIae - bIae;
  return finite(a?.metrics?.solveCount) - finite(b?.metrics?.solveCount);
}

function summarizeGroup(cases) {
  const ordered = [...cases].sort(compareExperimentCases);
  return {
    cases: cases.length,
    bestCaseId: ordered[0]?.id ?? null,
    worstCaseId: ordered[ordered.length - 1]?.id ?? null,
    meanIae: mean(cases.map((item) => item.metrics.iae)),
    worstIae: Math.max(...cases.map((item) => finite(item.metrics.iae))),
    meanSolveCount: mean(cases.map((item) => item.metrics.solveCount)),
    meanComputeReduction: mean(cases.map((item) => item.metrics.computeReduction)),
    meanConvergenceRate: mean(cases.map((item) => item.metrics.convergenceRate ?? 100)),
    totalFallbacks: cases.reduce((sum, item) => sum + finite(item.metrics.fallbackCount), 0),
    totalUnsafeSamples: cases.reduce((sum, item) => sum + finite(item.metrics.safetyViolationCount), 0),
    worstSafetyViolation: Math.max(...cases.map((item) => finite(item.metrics.maxActualSafetyViolation))),
    meanPositionRmse: mean(cases.map((item) => item.metrics.estimatePositionRmse)),
  };
}

export function runExperimentMatrix({
  presetIds = ['baseline'],
  seeds = [20260914],
  noiseScales = [1],
  mismatchScales = [1],
  mode = 'HYBRID_SAFE',
  baseConfig = defaultConfig,
  maxCases = 48,
} = {}) {
  const ids = [...new Set(presetIds)].filter(Boolean);
  const seedValues = seeds.length ? seeds : [baseConfig.estimation.seed];
  const noiseValues = noiseScales.length ? noiseScales : [1];
  const mismatchValues = mismatchScales.length ? mismatchScales : [1];
  const requestedCases = ids.length * seedValues.length * noiseValues.length * mismatchValues.length;
  if (!ids.length) throw new Error('Experiment matrix requires at least one preset.');
  if (requestedCases > maxCases) throw new Error(`Experiment matrix requests ${requestedCases} cases; limit is ${maxCases}.`);

  const cases = [];
  for (const presetId of ids) {
    const preset = getExperimentPreset(presetId);
    for (const seed of seedValues) {
      for (const noiseScale of noiseValues) {
        for (const mismatchScale of mismatchValues) {
          const presetConfig = applyExperimentPreset(baseConfig, presetId);
          const config = applyMatrixOverrides(presetConfig, { seed, noiseScale, mismatchScale });
          const result = runSimulation(mode, config);
          cases.push({
            id: `${presetId}:${seed}:${noiseScale}:${mismatchScale}`,
            presetId,
            presetLabel: preset.label,
            seed,
            noiseScale,
            mismatchScale,
            mode,
            metrics: { ...result.metrics },
            config,
            trace: compactTrace(result.samples),
            solverRecords: compactSolverRecords(result.solverRecords),
          });
        }
      }
    }
  }

  const groups = ids.map((presetId) => {
    const presetCases = cases.filter((item) => item.presetId === presetId);
    const preset = getExperimentPreset(presetId);
    return { presetId, presetLabel: preset.label, ...summarizeGroup(presetCases) };
  });

  return {
    createdAt: new Date().toISOString(),
    mode,
    requestedCases,
    dimensions: {
      presetIds: ids,
      seeds: [...seedValues],
      noiseScales: [...noiseValues],
      mismatchScales: [...mismatchValues],
    },
    cases,
    groups,
    overall: summarizeGroup(cases),
  };
}
