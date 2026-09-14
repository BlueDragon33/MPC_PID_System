import { createPIDController } from './controllers/pid.js';
import { createSecondOrderModel, disturbanceAt, stepSecondOrderPlant } from './models/secondOrderPlant.js';
import { solveMPC, SOLVER_BACKENDS } from './solvers/index.js';
import { evaluateEventTrigger } from './triggers/eventTrigger.js';

export const defaultConfig = {
  dt: 0.02,
  duration: 12,
  setpoint: 1,
  plant: { stiffness: 1.45, gain: 1.0, damping: 0.82 },
  pid: { kp: 5.2, ki: 1.35, kd: 0.52, uMin: -4, uMax: 4, antiWindup: 0.5 },
  mpc: {
    solver: SOLVER_BACKENDS.PROJECTED_GRADIENT,
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

export function getStateSpaceModel(cfg = defaultConfig) {
  return createSecondOrderModel(cfg);
}

function predictiveReference(solution, target, cfg) {
  const future = solution.path[solution.path.length - 1] ?? { x: target, v: 0 };
  const rawLead = cfg.mpc.referenceLead * (target - future.x) - cfg.mpc.velocityDamping * future.v;
  const lead = clamp(rawLead, -cfg.mpc.maxReferenceLead, cfg.mpc.maxReferenceLead);
  return target + lead;
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

function mergeConfig(userConfig) {
  return {
    ...defaultConfig,
    ...userConfig,
    plant: { ...defaultConfig.plant, ...(userConfig.plant || {}) },
    pid: { ...defaultConfig.pid, ...(userConfig.pid || {}) },
    mpc: { ...defaultConfig.mpc, ...(userConfig.mpc || {}) },
    trigger: { ...defaultConfig.trigger, ...(userConfig.trigger || {}) },
    disturbance: { ...defaultConfig.disturbance, ...(userConfig.disturbance || {}) },
  };
}

export function runSimulation(mode, userConfig = {}) {
  const cfg = mergeConfig(userConfig);
  const steps = Math.floor(cfg.duration / cfg.dt);
  const pid = createPIDController(cfg.pid, cfg.dt);
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
    const event = evaluateEventTrigger({
      k,
      t,
      state,
      expectedState,
      lastSolveState,
      previousU,
      lastSolve,
      cfg,
    });

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
    } else if (event.triggered) {
      const solution = solveMPC(state, cfg.setpoint, previousU, cfg, warmStart);
      warmStart = solution.sequence;
      reference = predictiveReference(solution, cfg.setpoint, cfg);
      lastSolve = t;
      lastSolveState = { ...state };
      solverTimes.push(solution.solveMs);
      mpcCost = solution.cost;
      triggered = true;
      triggerReason = event.reason;
      u = pid.update(reference, state.x);
    } else {
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
      predictionError: event.predictionError,
      stateChange: event.stateChange,
      constraintRatio: event.constraintRatio,
      triggered,
      triggerReason,
      mpcCost,
    });

    expectedState = stepSecondOrderPlant(state, u, 0, cfg);
    previousU = u;
    state = stepSecondOrderPlant(state, u, disturbance, cfg);
  }

  return {
    samples,
    metrics: metrics(samples, cfg.setpoint, solverTimes),
    config: cfg,
    model: createSecondOrderModel(cfg),
  };
}

export function compareControllers(config = {}) {
  return ['PID', 'MPC', 'HYBRID'].map((mode) => ({ mode, ...runSimulation(mode, config) }));
}
