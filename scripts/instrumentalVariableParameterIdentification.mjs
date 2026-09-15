import { defaultConfig } from '../src/core/simulator.js';
import { createSecondOrderModel, createTruthPlantConfig, stepSecondOrderPlant } from '../src/core/models/secondOrderPlant.js';
import { createMeasurementSensor } from '../src/core/estimation/measurementSensor.js';
import { createAugmentedDisturbanceKalmanFilter } from '../src/core/estimation/augmentedDisturbanceKalmanFilter.js';

const relativeError = (estimate, truth) => Math.abs(estimate - truth) / Math.max(Math.abs(truth), 1e-12);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);

function excitation(t) {
  return 1.2 * Math.sin(0.73 * t) + 0.8 * Math.sin(1.93 * t) + 0.35 * (Math.sin(0.31 * t) >= 0 ? 1 : -1);
}

function invert3(A) {
  const [a,b,c] = A[0];
  const [d,e,f] = A[1];
  const [g,h,i] = A[2];
  const A11 = e * i - f * h;
  const A12 = -(d * i - f * g);
  const A13 = d * h - e * g;
  const A21 = -(b * i - c * h);
  const A22 = a * i - c * g;
  const A23 = -(a * h - b * g);
  const A31 = b * f - c * e;
  const A32 = -(a * f - c * d);
  const A33 = a * e - b * d;
  const det = a * A11 + b * A12 + c * A13;
  if (!Number.isFinite(det) || Math.abs(det) < 1e-12) return null;
  const invDet = 1 / det;
  return [
    [A11 * invDet, A21 * invDet, A31 * invDet],
    [A12 * invDet, A22 * invDet, A32 * invDet],
    [A13 * invDet, A23 * invDet, A33 * invDet],
  ];
}

function matVec(A, x) {
  return A.map((row) => dot(row, x));
}

function solveRidgeNormal(samples, ridge = 0) {
  const M = [[ridge,0,0],[0,ridge,0],[0,0,ridge]];
  const q = [0,0,0];
  for (const s of samples) {
    for (let r = 0; r < 3; r += 1) {
      q[r] += s.phi[r] * s.y;
      for (let c = 0; c < 3; c += 1) M[r][c] += s.phi[r] * s.phi[c];
    }
  }
  const inv = invert3(M);
  return inv ? matVec(inv, q) : null;
}

function solveIV(samples, lag, ridge = 0) {
  const M = [[ridge,0,0],[0,ridge,0],[0,0,ridge]];
  const q = [0,0,0];
  let used = 0;
  for (let k = lag; k < samples.length; k += 1) {
    const s = samples[k];
    const z = samples[k - lag].phi;
    for (let r = 0; r < 3; r += 1) {
      q[r] += z[r] * s.y;
      for (let c = 0; c < 3; c += 1) M[r][c] += z[r] * s.phi[c];
    }
    used += 1;
  }
  const inv = invert3(M);
  return { theta: inv ? matVec(inv, q) : null, used };
}

const cfg = {
  ...defaultConfig,
  truthPlant: { enabled: true, stiffnessScale: 1.18, dampingScale: 0.78, gainScale: 0.90 },
};
const model = createSecondOrderModel(cfg);
const truthCfg = createTruthPlantConfig(cfg);
const truth = truthCfg.plant;

function collectSamples() {
  const sensor = createMeasurementSensor({ C: model.C, noiseStd: 0.03, seed: 20260915, bias: 0 });
  const estimator = createAugmentedDisturbanceKalmanFilter({
    A: model.A, B: model.B, E: model.E, C: model.C,
    processCovariance: [2e-5, 2e-4, 8e-3],
    measurementVariance: 0.03 ** 2,
    initialState: [0, 0, 0],
    initialCovariance: [0.04, 0.08, 0.2],
    disturbanceRetention: 0.90,
  });
  const initialMeasurement = sensor.read({ x: 0, v: 0 });
  let estimate = estimator.update(initialMeasurement.value);
  let truthState = { x: 0, v: 0 };
  const samples = [];
  const steps = Math.floor(30 / cfg.dt);

  for (let k = 0; k < steps; k += 1) {
    const t = k * cfg.dt;
    const u = excitation(t);
    const previous = { x: estimate.x, v: estimate.v };
    const nextTruth = stepSecondOrderPlant(truthState, u, 0, truthCfg);
    const measurement = sensor.read(nextTruth);
    estimate = estimator.step(u, measurement.value);
    samples.push({ phi: [previous.x, previous.v, u], y: estimate.v });
    truthState = nextTruth;
  }
  return samples;
}

function mapDiscrete(theta) {
  if (!theta) return null;
  const [a21, a22, b2] = theta;
  return {
    stiffness: -a21 / cfg.dt,
    damping: (1 - a22) / cfg.dt,
    gain: b2 / cfg.dt,
  };
}

function score(label, theta, used) {
  const p = mapDiscrete(theta);
  if (!p || !Object.values(p).every(Number.isFinite)) {
    return { method: label, used, stiffness: 'n/a', 'k err %': 'n/a', damping: 'n/a', 'c err %': 'n/a', gain: 'n/a', 'g err %': 'n/a', 'mean err %': 'inf' };
  }
  const errors = [relativeError(p.stiffness, truth.stiffness), relativeError(p.damping, truth.damping), relativeError(p.gain, truth.gain)];
  return {
    method: label,
    used,
    stiffness: p.stiffness.toFixed(4),
    'k err %': (100 * errors[0]).toFixed(2),
    damping: p.damping.toFixed(4),
    'c err %': (100 * errors[1]).toFixed(2),
    gain: p.gain.toFixed(4),
    'g err %': (100 * errors[2]).toFixed(2),
    'mean err %': (100 * errors.reduce((a,b) => a + b, 0) / 3).toFixed(2),
  };
}

const samples = collectSamples();
const rows = [];
for (const ridge of [0, 1e-6, 1e-4, 1e-2]) {
  rows.push(score(`OLS ridge=${ridge}`, solveRidgeNormal(samples, ridge), samples.length));
}
for (const lag of [1,2,3,4,5,8,12,20]) {
  for (const ridge of [0, 1e-6, 1e-4]) {
    const solved = solveIV(samples, lag, ridge);
    rows.push(score(`IV L${lag} ridge=${ridge}`, solved.theta, solved.used));
  }
}

console.log('Discrete-time instrumental-variable parameter-identification sweep');
console.log(`Truth: stiffness=${truth.stiffness.toFixed(4)}, damping=${truth.damping.toFixed(4)}, gain=${truth.gain.toFixed(4)}`);
console.table(rows);
const ranked = rows.filter((r) => Number.isFinite(Number(r['mean err %']))).sort((a,b) => Number(a['mean err %']) - Number(b['mean err %']));
if (ranked.length) console.log(`Best discrete/IV mean parameter error: ${ranked[0].method} = ${ranked[0]['mean err %']}%`);
