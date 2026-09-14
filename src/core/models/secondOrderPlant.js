export function createSecondOrderModel(cfg) {
  const dt = cfg.dt;
  const p = cfg.plant;
  return {
    A: [
      [1, dt],
      [-p.stiffness * dt, 1 - p.damping * dt],
    ],
    B: [0, p.gain * dt],
    E: [0, dt],
    C: [1, 0],
  };
}

export function stepSecondOrderPlant(state, u, disturbance, cfg) {
  const { A, B, E } = createSecondOrderModel(cfg);
  return {
    x: A[0][0] * state.x + A[0][1] * state.v + B[0] * u + E[0] * disturbance,
    v: A[1][0] * state.x + A[1][1] * state.v + B[1] * u + E[1] * disturbance,
  };
}

export function disturbanceAt(t, cfg) {
  const d = cfg.disturbance;
  if (!d?.enabled) return 0;
  if (t < d.start || t > d.start + d.duration) return 0;
  const phase = (t - d.start) / Math.max(d.duration, cfg.dt);
  return d.amplitude * (0.82 + 0.18 * Math.sin(Math.PI * phase));
}
