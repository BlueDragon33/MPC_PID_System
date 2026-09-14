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

export function createTruthPlantConfig(cfg) {
  const mismatch = cfg.truthPlant;
  if (!mismatch?.enabled) return cfg;
  return {
    ...cfg,
    plant: {
      ...cfg.plant,
      stiffness: cfg.plant.stiffness * (Number.isFinite(mismatch.stiffnessScale) ? mismatch.stiffnessScale : 1),
      damping: cfg.plant.damping * (Number.isFinite(mismatch.dampingScale) ? mismatch.dampingScale : 1),
      gain: cfg.plant.gain * (Number.isFinite(mismatch.gainScale) ? mismatch.gainScale : 1),
    },
  };
}

export function equivalentDisturbance(state, u, externalDisturbance, nominalCfg, truthCfg) {
  const nominal = createSecondOrderModel(nominalCfg);
  const truth = createSecondOrderModel(truthCfg);
  const e = truth.E[1];
  if (!Number.isFinite(e) || Math.abs(e) < 1e-12) return externalDisturbance;
  const residualVelocityIncrement =
    (truth.A[1][0] - nominal.A[1][0]) * state.x
    + (truth.A[1][1] - nominal.A[1][1]) * state.v
    + (truth.B[1] - nominal.B[1]) * u;
  return externalDisturbance + residualVelocityIncrement / e;
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
