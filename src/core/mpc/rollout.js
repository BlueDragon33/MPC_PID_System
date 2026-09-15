import { stepSecondOrderPlant } from '../models/secondOrderPlant.js';

const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

function disturbanceAtStage(cfg, stage) {
  if (!cfg.estimation?.mpcDisturbanceCompensationEnabled) return 0;
  const d0 = Number.isFinite(cfg.runtime?.disturbanceEstimate) ? cfg.runtime.disturbanceEstimate : 0;
  const rhoRaw = cfg.runtime?.disturbanceRetention ?? cfg.estimation?.disturbanceRetention ?? 1;
  const rho = clamp(Number.isFinite(rhoRaw) ? rhoRaw : 1, 0, 1);
  return d0 * (rho ** stage);
}

export function rolloutMPCSequence(state, sequence, target, previousU, cfg) {
  const xs = [{ ...state }];
  let cost = 0;
  let prev = previousU;

  for (let i = 0; i < sequence.length; i += 1) {
    const u = sequence[i];
    const disturbance = disturbanceAtStage(cfg, i);
    const next = stepSecondOrderPlant(xs[i], u, disturbance, cfg);
    xs.push(next);

    const terminal = i === sequence.length - 1 ? cfg.mpc.terminalWeight : 1;
    const positionError = next.x - target;
    cost += terminal * (
      cfg.mpc.qPosition * positionError * positionError
      + cfg.mpc.qVelocity * next.v * next.v
    );
    cost += cfg.mpc.rInput * u * u;
    cost += cfg.mpc.rDelta * (u - prev) * (u - prev);
    prev = u;
  }

  return { xs, cost };
}
