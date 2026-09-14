export const defaultConfig = {
  dt: 0.02,
  duration: 12,
  setpoint: 1,
  plant: { a: 1.45, b: 1.0, damping: 0.82 },
  pid: { kp: 5.2, ki: 1.35, kd: 0.52, uMin: -4, uMax: 4 },
  mpc: { horizon: 24, q: 7, r: 0.15, du: 0.08, uMin: -4, uMax: 4 },
  trigger: { stateError: 0.05, predictionError: 0.04, maxInterval: 0.28 },
};

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

function plantStep(state, u, cfg) {
  const { dt, plant } = cfg;
  const acceleration = plant.b * u - plant.a * state.x - plant.damping * state.v;
  return {
    x: state.x + dt * state.v,
    v: state.v + dt * acceleration,
  };
}

function makePID(params, dt) {
  let integral = 0;
  let prevError = 0;
  return {
    update(target, value) {
      const e = target - value;
      integral += e * dt;
      const derivative = (e - prevError) / dt;
      prevError = e;
      return clamp(params.kp * e + params.ki * integral + params.kd * derivative, params.uMin, params.uMax);
    },
  };
}

function predict(state, u, cfg, steps) {
  let s = { ...state };
  const path = [];
  for (let i = 0; i < steps; i += 1) {
    s = plantStep(s, u, cfg);
    path.push({ ...s });
  }
  return path;
}

function solveMPC(state, target, previousU, cfg) {
  const { mpc } = cfg;
  const candidates = 41;
  let best = { u: 0, cost: Number.POSITIVE_INFINITY, path: [] };

  for (let i = 0; i < candidates; i += 1) {
    const u = mpc.uMin + (i / (candidates - 1)) * (mpc.uMax - mpc.uMin);
    const path = predict(state, u, cfg, mpc.horizon);
    let cost = 0;
    path.forEach((p, k) => {
      const e = target - p.x;
      const terminalWeight = k === path.length - 1 ? 2.2 : 1;
      cost += terminalWeight * mpc.q * e * e + mpc.r * u * u;
    });
    const deltaU = u - previousU;
    cost += mpc.du * deltaU * deltaU;
    if (cost < best.cost) best = { u, cost, path };
  }
  return best;
}

function settlingTime(samples, target, tolerance = 0.02) {
  for (let i = 0; i < samples.length; i += 1) {
    const tail = samples.slice(i);
    if (tail.every((p) => Math.abs(p.x - target) <= tolerance)) return samples[i].t;
  }
  return null;
}

function metrics(samples, target, solveCount = 0) {
  const peak = Math.max(...samples.map((p) => p.x));
  const iae = samples.reduce((sum, p) => sum + Math.abs(target - p.x) * (samples[1]?.t ?? 0.02), 0);
  const controlEffort = samples.reduce((sum, p) => sum + Math.abs(p.u) * (samples[1]?.t ?? 0.02), 0);
  return {
    overshoot: Math.max(0, ((peak - target) / Math.max(Math.abs(target), 1e-9)) * 100),
    settling: settlingTime(samples, target),
    iae,
    controlEffort,
    solveCount,
  };
}

export function runSimulation(mode, userConfig = {}) {
  const cfg = {
    ...defaultConfig,
    ...userConfig,
    plant: { ...defaultConfig.plant, ...(userConfig.plant || {}) },
    pid: { ...defaultConfig.pid, ...(userConfig.pid || {}) },
    mpc: { ...defaultConfig.mpc, ...(userConfig.mpc || {}) },
    trigger: { ...defaultConfig.trigger, ...(userConfig.trigger || {}) },
  };
  const steps = Math.floor(cfg.duration / cfg.dt);
  const pid = makePID(cfg.pid, cfg.dt);
  let state = { x: 0, v: 0 };
  let previousU = 0;
  let lastSolve = -Infinity;
  let predictedNext = { ...state };
  let reference = cfg.setpoint;
  let solveCount = 0;
  const samples = [];

  for (let k = 0; k <= steps; k += 1) {
    const t = k * cfg.dt;
    let u = previousU;
    let triggered = false;
    let triggerReason = '';

    if (mode === 'PID') {
      u = pid.update(cfg.setpoint, state.x);
    } else if (mode === 'MPC') {
      const solution = solveMPC(state, cfg.setpoint, previousU, cfg);
      u = solution.u;
      predictedNext = solution.path[0] || state;
      solveCount += 1;
      triggered = true;
      triggerReason = 'periodic';
    } else {
      const stateError = Math.abs(cfg.setpoint - state.x);
      const predictionError = Math.abs(state.x - predictedNext.x);
      const elapsed = t - lastSolve;
      const mustSolve = k === 0 || predictionError >= cfg.trigger.predictionError || (stateError >= cfg.trigger.stateError && elapsed >= cfg.trigger.maxInterval) || elapsed >= cfg.trigger.maxInterval * 2;

      if (mustSolve) {
        const solution = solveMPC(state, cfg.setpoint, previousU, cfg);
        const previewIndex = Math.min(7, solution.path.length - 1);
        reference = solution.path[previewIndex]?.x ?? cfg.setpoint;
        predictedNext = solution.path[0] || state;
        lastSolve = t;
        solveCount += 1;
        triggered = true;
        triggerReason = predictionError >= cfg.trigger.predictionError ? 'prediction-error' : elapsed >= cfg.trigger.maxInterval * 2 ? 'watchdog' : 'state-change';
      } else {
        predictedNext = plantStep(predictedNext, previousU, cfg);
      }
      u = pid.update(reference, state.x);
    }

    samples.push({ t, x: state.x, v: state.v, u, reference, triggered, triggerReason });
    previousU = u;
    state = plantStep(state, u, cfg);
  }

  return { samples, metrics: metrics(samples, cfg.setpoint, solveCount), config: cfg };
}

export function compareControllers(config = {}) {
  return ['PID', 'MPC', 'HYBRID'].map((mode) => ({ mode, ...runSimulation(mode, config) }));
}
