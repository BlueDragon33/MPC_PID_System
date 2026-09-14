import { stepSecondOrderPlant } from '../models/secondOrderPlant.js';

export function rolloutMPCSequence(state, sequence, target, previousU, cfg) {
  const xs = [{ ...state }];
  let cost = 0;
  let prev = previousU;

  for (let i = 0; i < sequence.length; i += 1) {
    const u = sequence[i];
    const next = stepSecondOrderPlant(xs[i], u, 0, cfg);
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
