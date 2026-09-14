import { createSecondOrderModel, stepSecondOrderPlant } from '../models/secondOrderPlant.js';

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const now = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

function rollout(state, sequence, target, previousU, cfg) {
  const xs = [{ ...state }];
  let cost = 0;
  let prev = previousU;

  for (let i = 0; i < sequence.length; i += 1) {
    const u = sequence[i];
    const next = stepSecondOrderPlant(xs[i], u, 0, cfg);
    xs.push(next);
    const terminal = i === sequence.length - 1 ? cfg.mpc.terminalWeight : 1;
    const e = next.x - target;
    cost += terminal * (cfg.mpc.qPosition * e * e + cfg.mpc.qVelocity * next.v * next.v);
    cost += cfg.mpc.rInput * u * u + cfg.mpc.rDelta * (u - prev) * (u - prev);
    prev = u;
  }

  return { xs, cost };
}

export function evaluateMPCSequenceCost(state, sequence, target, previousU, cfg) {
  return rollout(state, sequence, target, previousU, cfg).cost;
}

function stateGradient(state, target, cfg, weight = 1) {
  return [
    2 * weight * cfg.mpc.qPosition * (state.x - target),
    2 * weight * cfg.mpc.qVelocity * state.v,
  ];
}

function sequenceGradient(xs, us, target, previousU, cfg) {
  const { A, B } = createSecondOrderModel(cfg);
  const n = us.length;
  const grad = new Array(n).fill(0);
  let lambda = stateGradient(xs[n], target, cfg, cfg.mpc.terminalWeight);

  for (let i = n - 1; i >= 0; i -= 1) {
    const prev = i === 0 ? previousU : us[i - 1];
    const next = i < n - 1 ? us[i + 1] : null;
    let g = B[0] * lambda[0] + B[1] * lambda[1] + 2 * cfg.mpc.rInput * us[i];
    g += 2 * cfg.mpc.rDelta * (us[i] - prev);
    if (next !== null) g -= 2 * cfg.mpc.rDelta * (next - us[i]);
    grad[i] = g;

    if (i > 0) {
      const local = stateGradient(xs[i], target, cfg, 1);
      lambda = [
        A[0][0] * lambda[0] + A[1][0] * lambda[1] + local[0],
        A[0][1] * lambda[0] + A[1][1] * lambda[1] + local[1],
      ];
    }
  }

  return grad;
}

export function solveProjectedGradientMPC(state, target, previousU, cfg, warmStart = null) {
  const started = now();
  const n = cfg.mpc.horizon;
  let us = warmStart?.length === n
    ? [...warmStart.slice(1), warmStart[warmStart.length - 1]]
    : new Array(n).fill(previousU);

  for (let iter = 0; iter < cfg.mpc.iterations; iter += 1) {
    const { xs } = rollout(state, us, target, previousU, cfg);
    const grad = sequenceGradient(xs, us, target, previousU, cfg);
    const step = cfg.mpc.learningRate / (1 + iter * 0.035);
    for (let i = 0; i < us.length; i += 1) {
      us[i] = clamp(us[i] - step * grad[i], cfg.mpc.uMin, cfg.mpc.uMax);
    }
  }

  const result = rollout(state, us, target, previousU, cfg);
  return {
    u: us[0],
    sequence: us,
    path: result.xs.slice(1),
    cost: result.cost,
    solveMs: now() - started,
    solver: 'projected-gradient',
  };
}
