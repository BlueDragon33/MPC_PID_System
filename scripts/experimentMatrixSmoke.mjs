import assert from 'node:assert/strict';
import { compareExperimentCases, runExperimentMatrix } from '../src/core/experiments/experimentMatrix.js';
import { defaultConfig } from '../src/core/simulator.js';

const baseConfig = {
  ...defaultConfig,
  duration: 1.2,
  disturbance: { ...defaultConfig.disturbance, enabled: false },
  mpc: { ...defaultConfig.mpc, horizon: 14, qpIterations: 80, qpProjectionCycles: 10 },
};

const singleA = runExperimentMatrix({ presetIds: ['baseline'], seeds: [17], baseConfig });
const singleB = runExperimentMatrix({ presetIds: ['baseline'], seeds: [17], baseConfig });
assert.equal(singleA.requestedCases, 1);
assert.equal(singleA.cases[0].metrics.iae, singleB.cases[0].metrics.iae, 'same matrix seed should reproduce IAE');
assert.equal(singleA.cases[0].metrics.solveCount, singleB.cases[0].metrics.solveCount, 'same matrix seed should reproduce solve count');
assert(singleA.cases[0].trace.length > 2, 'case explorer should retain compact trajectory');
assert(Array.isArray(singleA.cases[0].solverRecords), 'case explorer should retain solver audit');
assert('t' in singleA.cases[0].trace[0] && 'x' in singleA.cases[0].trace[0] && 'u' in singleA.cases[0].trace[0]);
assert(!('samples' in singleA.cases[0]), 'matrix should keep compact traces rather than duplicate full simulation objects');

const matrix = runExperimentMatrix({
  presetIds: ['baseline', 'noisy-estimation'],
  seeds: [17, 23],
  noiseScales: [1],
  mismatchScales: [1],
  baseConfig,
});
assert.equal(matrix.requestedCases, 4, 'matrix should produce Cartesian product');
assert.equal(matrix.cases.length, 4);
assert.equal(matrix.groups.length, 2);
assert(matrix.groups.every((group) => group.cases === 2));
assert(matrix.groups.every((group) => matrix.cases.some((item) => item.id === group.bestCaseId)));
assert(matrix.groups.every((group) => matrix.cases.some((item) => item.id === group.worstCaseId)));
assert(Number.isFinite(matrix.overall.meanIae));
assert(Number.isFinite(matrix.overall.meanSolveCount));
assert(matrix.cases.every((item) => item.mode === 'HYBRID_SAFE'));

const manualMeanIae = matrix.cases.reduce((sum, item) => sum + item.metrics.iae, 0) / matrix.cases.length;
assert(Math.abs(manualMeanIae - matrix.overall.meanIae) < 1e-12, 'aggregate IAE should match case-level mean');

const saferButWorseIae = { metrics: { safetyViolationCount: 0, maxActualSafetyViolation: 0, fallbackCount: 0, iae: 2, solveCount: 20 } };
const unsafeButBetterIae = { metrics: { safetyViolationCount: 1, maxActualSafetyViolation: 0.01, fallbackCount: 0, iae: 0.5, solveCount: 10 } };
assert(compareExperimentCases(saferButWorseIae, unsafeButBetterIae) < 0, 'robustness ordering must prioritize safety over IAE');

assert.throws(() => runExperimentMatrix({
  presetIds: ['baseline', 'noisy-estimation'],
  seeds: [1,2,3],
  noiseScales: [0.5,1,1.5],
  mismatchScales: [0.5,1,1.5],
  baseConfig,
  maxCases: 48,
}), /limit is 48/);

console.log('experiment matrix smoke: PASS', {
  cases: matrix.requestedCases,
  meanIae: matrix.overall.meanIae,
  meanSolveCount: matrix.overall.meanSolveCount,
  compactTraceSamples: matrix.cases[0].trace.length,
});
