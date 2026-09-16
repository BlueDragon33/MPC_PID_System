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

function summarizeGroup(cases) {
  return {
    cases: cases.length,
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
