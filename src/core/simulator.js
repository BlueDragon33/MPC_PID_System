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
    solver: SOLVER_BACKENDS.CONSTRAINED_QP,
    horizon: 35,
    qPosition: 9,
    qVelocity: 1.2,
    rInput: 0.16,
    rDelta: 0.12,
    terminalWeight: 8,
    iterations: 20,
    learningRate: 0.12,
    qpIterations: 60,
    qpTolerance: 5e-4,
    qpStepScale: 0.95,
    qpProjectionCycles: 6,
    qpProjectionTolerance: 1e-9,
    qpFeasibilityTolerance: 1e-7,
    qpTimeBudgetMs: 0,
    uMin: -4,
    uMax: 4,
    deltaUMin: -0.65,
    deltaUMax: 0.65,
    stateConstraintsEnabled: false,
    positionMin: -1.5,
    positionMax: 1.5,
    velocityMin: -1.2,
    velocityMax: 1.2,
    outputConstraintsEnabled: false,
    outputMin: -0.2,
    outputMax: 1.05,
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
const average = (values) => values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0;

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

function safetyViolationAt(sample, cfg) {
  let violation = 0;
  if (cfg.mpc.stateConstraintsEnabled) {
    if (Number.isFinite(cfg.mpc.positionMin)) violation = Math.max(violation, cfg.mpc.positionMin - sample.x);
    if (Number.isFinite(cfg.mpc.positionMax)) violation = Math.max(violation, sample.x - cfg.mpc.positionMax);
    if (Number.isFinite(cfg.mpc.velocityMin)) violation = Math.max(violation, cfg.mpc.velocityMin - sample.v);
    if (Number.isFinite(cfg.mpc.velocityMax)) violation = Math.max(violation, sample.v - cfg.mpc.velocityMax);
  }
  if (cfg.mpc.outputConstraintsEnabled) {
    const y = sample.x;
    if (Number.isFinite(cfg.mpc.outputMin)) violation = Math.max(violation, cfg.mpc.outputMin - y);
    if (Number.isFinite(cfg.mpc.outputMax)) violation = Math.max(violation, y - cfg.mpc.outputMax);
  }
  return Math.max(0, violation);
}

function metrics(samples, target, solverRecords, cfg) {
  const dt = samples[1]?.t ?? 0.02;
  const peak = Math.max(...samples.map((p) => p.x));
  const solveCount = solverRecords.length;
  const solverTimes = solverRecords.map((record) => record.solveMs);
  const periodicEquivalent = Math.max(1, samples.length);
  const totalSolveMs = solverTimes.reduce((a, b) => a + b, 0);
  const diagnostics = solverRecords.map((record) => record.diagnostics).filter(Boolean);
  const convergenceDiagnostics = diagnostics.filter((item) => typeof item.converged === 'boolean');
  const residualValues = diagnostics
    .map((item) => item.projectedGradientResidual ?? item.kktResidual)
    .filter(Number.isFinite);
  const feasibilityValues = diagnostics.map((item) => item.feasibilityViolation).filter(Number.isFinite);
  const iterationValues = diagnostics.map((item) => item.iterations).filter(Number.isFinite);
  const activeRatios = diagnostics.map((item) => item.activeConstraintRatio).filter(Number.isFinite);
  const statuses = solverRecords.map((record) => record.status).filter(Boolean);
  const fallbackCount = solverRecords.filter((record) => record.fallbackUsed).length;
  const approximateCount = diagnostics.filter((item) => item.acceptedApproximate).length;
  const safetyViolations = samples.map((sample) => safetyViolationAt(sample, cfg));
  const safetyViolationCount = safetyViolations.filter((value) => value > 1e-9).length;
  const stateActiveSolves = diagnostics.filter((item) => {
    const active = item.activeByKind || {};
    return Object.keys(active).some((key) => (key.startsWith('state-') || key.startsWith('output-')) && active[key] > 0);
  }).length;

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
    convergenceRate: convergenceDiagnostics.length
      ? 100 * convergenceDiagnostics.filter((item) => item.converged).length / convergenceDiagnostics.length
      : null,
    avgIterations: average(iterationValues),
    avgStationarityResidual: average(residualValues),
    maxStationarityResidual: residualValues.length ? Math.max(...residualValues) : null,
    maxFeasibilityViolation: feasibilityValues.length ? Math.max(...feasibilityValues) : null,
    avgActiveConstraintRatio: average(activeRatios),
    fallbackCount,
    fallbackRate: solveCount ? 100 * fallbackCount / solveCount : 0,
    approximateCount,
    timeoutCount: statuses.filter((status) => status === 'timeout').length,
    infeasibleCount: statuses.filter((status) => status === 'infeasible').length,
    numericalFailureCount: statuses.filter((status) => status === 'numerical-failure').length,
    maxActualSafetyViolation: safetyViolations.length ? Math.max(...safetyViolations) : 0,
    safetyViolationCount,
    safetyViolationRate: samples.length ? 100 * safetyViolationCount / samples.length : 0,
    stateConstraintActiveSolveRate: solveCount ? 100 * stateActiveSolves / solveCount : 0,
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
  const solverRecords = [];
  const samples = [];

  function recordSolution(solution) {
    solverRecords.push({
      solveMs: solution.solveMs,
      solver: solution.solver,
      status: solution.status || (solution.diagnostics?.converged ? 'solved' : null),
      fallbackUsed: Boolean(solution.fallbackUsed),
      fallbackReason: solution.fallbackReason || null,
      diagnostics: solution.diagnostics || null,
    });
  }

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
    let solverDiagnostics = null;
    let solverStatus = null;
    let fallbackUsed = false;

    if (mode === 'PID') {
      u = pid.update(cfg.setpoint, state.x);
    } else if (mode === 'MPC') {
      const solution = solveMPC(state, cfg.setpoint, previousU, cfg, warmStart);
      u = solution.u;
      warmStart = solution.sequence;
      mpcCost = solution.cost;
      solverDiagnostics = solution.diagnostics || null;
      solverStatus = solution.status || null;
      fallbackUsed = Boolean(solution.fallbackUsed);
      recordSolution(solution);
      triggered = true;
      triggerReason = 'periodic';
    } else if (event.triggered) {
      const solution = solveMPC(state, cfg.setpoint, previousU, cfg, warmStart);
      warmStart = solution.sequence;
      if (!solution.fallbackUsed) reference = predictiveReference(solution, cfg.setpoint, cfg);
      lastSolve = t;
      lastSolveState = { ...state };
      mpcCost = solution.cost;
      solverDiagnostics = solution.diagnostics || null;
      solverStatus = solution.status || null;
      fallbackUsed = Boolean(solution.fallbackUsed);
      recordSolution(solution);
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
      solverStatus,
      fallbackUsed,
      solverConverged: solverDiagnostics?.converged ?? null,
      solverIterations: solverDiagnostics?.iterations ?? null,
      stationarityResidual: solverDiagnostics?.projectedGradientResidual ?? solverDiagnostics?.kktResidual ?? null,
      feasibilityViolation: solverDiagnostics?.feasibilityViolation ?? null,
      activeConstraintRatio: solverDiagnostics?.activeConstraintRatio ?? null,
      safetyViolation: safetyViolationAt(state, cfg),
    });

    expectedState = stepSecondOrderPlant(state, u, 0, cfg);
    previousU = u;
    state = stepSecondOrderPlant(state, u, disturbance, cfg);
  }

  return {
    samples,
    metrics: metrics(samples, cfg.setpoint, solverRecords, cfg),
    config: cfg,
    model: createSecondOrderModel(cfg),
    solverRecords,
  };
}

export function compareControllers(config = {}) {
  return ['PID', 'MPC', 'HYBRID'].map((mode) => ({ mode, ...runSimulation(mode, config) }));
}
