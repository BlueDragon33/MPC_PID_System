export const defaultConfig = {
  dt: 0.02,
  duration: 12,
  setpoint: 1,
  plant: { stiffness: 1.45, gain: 1.0, damping: 0.82 },
  pid: { kp: 5.2, ki: 1.35, kd: 0.52, uMin: -4, uMax: 4, antiWindup: 0.5 },
  mpc: {
    horizon: 35,
    qPosition: 9,
    qVelocity: 1.2,
    rInput: 0.16,
    rDelta: 0.12,
    terminalWeight: 8,
    iterations: 20,
    learningRate: 0.12,
    uMin: -4,
    uMax: 4,
    referenceLead: 0.5,
    velocityDamping: 0.08,
    maxReferenceLead: 0.4,
  },
  trigger: {
    predictionError: 0.035,
    stateChange: 0.08,
    minInterval: 0.08,
    maxInterval: 0.36,
    constraintRatio: 0.92,
    positionScale: 1,
    velocityScale: 1,
  },
  disturbance: { enabled: true, start: 4.0, duration: 1.0, amplitude: 1.6 },
};

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const now = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

export function getStateSpaceModel(cfg = defaultConfig) {
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

function modelStep(state, u, disturbance, cfg) {
  const { A, B, E } = getStateSpaceModel(cfg);
  return {
    x: A[0][0] * state.x + A[0][1] * state.v + B[0] * u + E[0] * disturbance,
    v: A[1][0] * state.x + A[1][1] * state.v + B[1] * u + E[1] * disturbance,
  };
}

function disturbanceAt(t, cfg) {
  const d = cfg.disturbance;
  if (!d?.enabled) return 0;
  if (t < d.start || t > d.start + d.duration) return 0;
  const phase = (t - d.start) / Math.max(d.duration, cfg.dt);
  return d.amplitude * (0.82 + 0.18 * Math.sin(Math.PI * phase));
}

function makePID(params, dt) {
  let integral = 0;
  let prevError = 0;
  return {
    update(target, value) {
      const e = target - value;
      const derivative = (e - prevError) / dt;
      const rawIntegral = integral + e * dt;
      const raw = params.kp * e + params.ki * rawIntegral + params.kd * derivative;
      const u = clamp(raw, params.uMin, params.uMax);
      const saturated = Math.abs(raw - u) > 1e-10;
      integral = saturated ? integral + params.antiWindup * e * dt : rawIntegral;
      prevError = e;
      return u;
    },
  };
}

function rollout(state, sequence, target, previousU, cfg) {
  const xs = [{ ...state }];
  let cost = 0;
  let prev = previousU;
  for (let i = 0; i < sequence.length; i += 1) {
    const u = sequence[i];
    const next = modelStep(xs[i], u, 0, cfg);
    xs.push(next);
    const terminal = i === sequence.length - 1 ? cfg.mpc.terminalWeight : 1;
    const e = next.x - target;
    cost += terminal * (cfg.mpc.qPosition * e * e + cfg.mpc.qVelocity * next.v * next.v);
    cost += cfg.mpc.rInput * u * u + cfg.mpc.rDelta * (u - prev) * (u - prev);
    prev = u;
  }
  return { xs, cost };
}

function stateGradient(state, target, cfg, weight = 1) {
  return [
    2 * weight * cfg.mpc.qPosition * (state.x - target),
    2 * weight * cfg.mpc.qVelocity * state.v,
  ];
}

function sequenceGradient(xs, us, target, previousU, cfg) {
  const { A, B } = getStateSpaceModel(cfg);
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

function solveMPC(state, target, previousU, cfg, warmStart = null) {
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
  };
}

function predictiveReference(solution, target, cfg) {
  const future = solution.path[solution.path.length - 1] ?? { x: target, v: 0 };
  const rawLead = cfg.mpc.referenceLead * (target - future.x) - cfg.mpc.velocityDamping * future.v;
  const lead = clamp(rawLead, -cfg.mpc.maxReferenceLead, cfg.mpc.maxReferenceLead);
  return target + lead;
}

function normalizedDistance(a, b, trigger) {
  const dx = (a.x - b.x) / Math.max(trigger.positionScale, 1e-9);
  const dv = (a.v - b.v) / Math.max(trigger.velocityScale, 1e-9);
  return Math.sqrt(dx * dx + dv * dv);
}

function settlingTime(samples, target, tolerance = 0.02) {
  for (let i = 0; i < samples.length; i += 1) {
    if (samples.slice(i).every((p) => Math.abs(p.x - target) <= tolerance)) return samples[i].t;
  }
  return null;
}

function metrics(samples, target, solverTimes) {
  const dt = samples[1]?.t ?? 0.02;
  const peak = Math.max(...samples.map((p) => p.x));
  const solveCount = solverTimes.length;
  const periodicEquivalent = Math.max(1, samples.length);
  const totalSolveMs = solverTimes.reduce((a, b) => a + b, 0);
  return {
    overshoot: Math.max(0, ((peak - target) / Math.max(Math.abs(target), 1e-9)) * 100),
    settling: settlingTime(samples, target),
    iae: samples.reduce((sum, p) => sum + Math.abs(target - p.x) * dt, 0),
    controlEffort: samples.reduce((sum, p) => sum + Math.abs(p.u) * dt, 0),
    solveCount,
    avgSolveMs: solveCount ? totalSolveMs / solveCount : 0,
    maxSolveMs: solveCount ? Math.max(...solverTimes) : 0,
    totalSolveMs,
    computeReduction: 100 * (1 - solveCount / periodicEquivalent),
    triggerRate: solveCount / Math.max(samples[samples.length - 1]?.t ?? 1, 1e-9),
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
    disturbance: { ...defaultConfig.disturbance, ...(userConfig.disturbance || {}) },
  };

  const steps = Math.floor(cfg.duration / cfg.dt);
  const pid = makePID(cfg.pid, cfg.dt);
  let state = { x: 0, v: 0 };
  let expectedState = { ...state };
  let lastSolveState = { ...state };
  let previousU = 0;
  let lastSolve = -Infinity;
  let reference = cfg.setpoint;
  let warmStart = null;
  const solverTimes = [];
  const samples = [];

  for (let k = 0; k <= steps; k += 1) {
    const t = k * cfg.dt;
    const predictionError = k === 0 ? 0 : normalizedDistance(state, expectedState, cfg.trigger);
    const stateChange = normalizedDistance(state, lastSolveState, cfg.trigger);
    const elapsed = t - lastSolve;
    const constraintRatio = Math.abs(previousU) / Math.max(Math.abs(cfg.pid.uMax), 1e-9);
    let u = previousU;
    let triggered = false;
    let triggerReason = '';
    let mpcCost = null;

    if (mode === 'PID') {
      u = pid.update(cfg.setpoint, state.x);
    } else if (mode === 'MPC') {
      const solution = solveMPC(state, cfg.setpoint, previousU, cfg, warmStart);
      u = solution.u;
      warmStart = solution.sequence;
      mpcCost = solution.cost;
      solverTimes.push(solution.solveMs);
      triggered = true;
      triggerReason = 'periodic';
    } else {
      const intervalReady = elapsed >= cfg.trigger.minInterval;
      const watchdog = elapsed >= cfg.trigger.maxInterval;
      const predictionEvent = intervalReady && predictionError >= cfg.trigger.predictionError;
      const stateEvent = intervalReady && stateChange >= cfg.trigger.stateChange;
      const constraintEvent = intervalReady && constraintRatio >= cfg.trigger.constraintRatio;
      const mustSolve = k === 0 || watchdog || predictionEvent || stateEvent || constraintEvent;

      if (mustSolve) {
        const solution = solveMPC(state, cfg.setpoint, previousU, cfg, warmStart);
        warmStart = solution.sequence;
        reference = predictiveReference(solution, cfg.setpoint, cfg);
        lastSolve = t;
        lastSolveState = { ...state };
        solverTimes.push(solution.solveMs);
        mpcCost = solution.cost;
        triggered = true;
        triggerReason = k === 0 ? 'initial' : watchdog ? 'watchdog' : predictionEvent ? 'prediction-error' : constraintEvent ? 'constraint' : 'state-change';
      }

      u = pid.update(reference, state.x);
    }

    const disturbance = disturbanceAt(t, cfg);
    samples.push({
      t,
      x: state.x,
      v: state.v,
      u,
      reference,
      disturbance,
      predictionError,
      stateChange,
      triggered,
      triggerReason,
      mpcCost,
    });

    expectedState = modelStep(state, u, 0, cfg);
    previousU = u;
    state = modelStep(state, u, disturbance, cfg);
  }

  return { samples, metrics: metrics(samples, cfg.setpoint, solverTimes), config: cfg, model: getStateSpaceModel(cfg) };
}

export function compareControllers(config = {}) {
  return ['PID', 'MPC', 'HYBRID'].map((mode) => ({ mode, ...runSimulation(mode, config) }));
}
