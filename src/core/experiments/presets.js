export const EXPERIMENT_PRESETS = [
  {
    id: 'baseline',
    label: 'Baseline hybrid',
    description: 'Default event-triggered MPC + PID with actuator and slew-rate constraints.',
    patch: {
      mpc: {
        stateConstraintsEnabled: false,
        outputConstraintsEnabled: false,
        deltaUMin: -0.65,
        deltaUMax: 0.65,
      },
      disturbance: { enabled: true, start: 4.0, duration: 1.0, amplitude: 1.6 },
    },
  },
  {
    id: 'rate-limited',
    label: 'Tight actuator slew rate',
    description: 'Highlights the difference between penalizing Δu and enforcing it as a hard physical limit.',
    patch: {
      mpc: {
        stateConstraintsEnabled: false,
        outputConstraintsEnabled: false,
        deltaUMin: -0.25,
        deltaUMax: 0.25,
        rDelta: 0.08,
      },
      disturbance: { enabled: true, start: 4.0, duration: 1.0, amplitude: 1.4 },
    },
  },
  {
    id: 'safety-envelope',
    label: 'Predicted safety envelope',
    description: 'Constrains future velocity and measured output so MPC must anticipate the safe region before PID reacts.',
    patch: {
      safety: {
        previewHorizon: 8,
        positionMargin: 0,
        velocityMargin: 0,
        outputMargin: 0,
      },
      mpc: {
        stateConstraintsEnabled: true,
        positionMin: -1.5,
        positionMax: 1.5,
        velocityMin: -0.8,
        velocityMax: 0.8,
        outputConstraintsEnabled: true,
        outputMin: -0.2,
        outputMax: 1.04,
        deltaUMin: -0.4,
        deltaUMax: 0.4,
        qpProjectionCycles: 14,
      },
      disturbance: { enabled: true, start: 4.0, duration: 1.0, amplitude: 1.2 },
    },
  },
  {
    id: 'disturbance-stress',
    label: 'Disturbance stress test',
    description: 'Large plant-only disturbance used to study trigger density, recovery and constraint activation.',
    patch: {
      safety: {
        previewHorizon: 10,
        positionMargin: 0,
        velocityMargin: 0,
        outputMargin: 0,
      },
      mpc: {
        stateConstraintsEnabled: true,
        positionMin: -1.8,
        positionMax: 1.8,
        velocityMin: -1.25,
        velocityMax: 1.25,
        outputConstraintsEnabled: true,
        outputMin: -0.35,
        outputMax: 1.12,
        deltaUMin: -0.5,
        deltaUMax: 0.5,
        qpProjectionCycles: 16,
      },
      trigger: {
        predictionError: 0.025,
        stateChange: 0.06,
        maxInterval: 0.3,
      },
      disturbance: { enabled: true, start: 3.4, duration: 1.5, amplitude: 2.4 },
    },
  },
  {
    id: 'infeasible-guard',
    label: 'Infeasible-envelope guard',
    description: 'Deliberately impossible first-step velocity envelope; validates explicit infeasibility and safe actuator fallback.',
    patch: {
      duration: 2.0,
      safety: { previewHorizon: 6 },
      mpc: {
        stateConstraintsEnabled: true,
        positionMin: -1.5,
        positionMax: 1.5,
        velocityMin: 1.0,
        velocityMax: 1.1,
        outputConstraintsEnabled: false,
        deltaUMin: -0.3,
        deltaUMax: 0.3,
        qpProjectionCycles: 18,
      },
      disturbance: { enabled: false },
    },
  },
];

export function getExperimentPreset(id) {
  return EXPERIMENT_PRESETS.find((preset) => preset.id === id) || EXPERIMENT_PRESETS[0];
}

export function applyExperimentPreset(baseConfig, id) {
  const preset = getExperimentPreset(id);
  const patch = preset.patch;
  return {
    ...baseConfig,
    ...patch,
    plant: { ...baseConfig.plant, ...(patch.plant || {}) },
    pid: { ...baseConfig.pid, ...(patch.pid || {}) },
    mpc: { ...baseConfig.mpc, ...(patch.mpc || {}) },
    safety: { ...baseConfig.safety, ...(patch.safety || {}) },
    trigger: { ...baseConfig.trigger, ...(patch.trigger || {}) },
    disturbance: { ...baseConfig.disturbance, ...(patch.disturbance || {}) },
  };
}
