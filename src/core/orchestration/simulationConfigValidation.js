import { SOLVER_BACKENDS } from '../solvers/index.js';

export const MAX_SIMULATION_STEPS = 1_000_000;

export class SimulationConfigValidationError extends Error {
  constructor(issues) {
    super(`Invalid simulation configuration: ${issues.join('; ')}`);
    this.name = 'SimulationConfigValidationError';
    this.issues = [...issues];
  }
}

const isObject = (value) => value && typeof value === 'object' && !Array.isArray(value);
const isFiniteNumber = (value) => typeof value === 'number' && Number.isFinite(value);
const isExtendedNumber = (value) => typeof value === 'number' && !Number.isNaN(value);

function validateNumber(issues, path, value, {
  min = null,
  max = null,
  integer = false,
  strictMin = false,
  strictMax = false,
  extended = false,
} = {}) {
  const validNumber = extended ? isExtendedNumber(value) : isFiniteNumber(value);
  if (!validNumber) {
    issues.push(`${path} must be ${extended ? 'a number' : 'finite'}`);
    return;
  }
  if (integer && !Number.isInteger(value)) issues.push(`${path} must be an integer`);
  if (min != null && (strictMin ? value <= min : value < min)) {
    issues.push(`${path} must be ${strictMin ? '>' : '>='} ${min}`);
  }
  if (max != null && (strictMax ? value >= max : value > max)) {
    issues.push(`${path} must be ${strictMax ? '<' : '<='} ${max}`);
  }
}

function validateBoolean(issues, path, value) {
  if (typeof value !== 'boolean') issues.push(`${path} must be boolean`);
}

function validateOrdered(issues, minPath, minValue, maxPath, maxValue, { extended = false } = {}) {
  const validMin = extended ? isExtendedNumber(minValue) : isFiniteNumber(minValue);
  const validMax = extended ? isExtendedNumber(maxValue) : isFiniteNumber(maxValue);
  if (validMin && validMax && minValue > maxValue) {
    issues.push(`${minPath} must be <= ${maxPath}`);
  }
}

export function validateSimulationConfig(cfg) {
  const issues = [];
  if (!isObject(cfg)) return ['config must be an object'];

  validateNumber(issues, 'dt', cfg.dt, { min: 0, strictMin: true });
  validateNumber(issues, 'duration', cfg.duration, { min: 0, strictMin: true });
  validateNumber(issues, 'setpoint', cfg.setpoint);

  if (isFiniteNumber(cfg.dt) && cfg.dt > 0 && isFiniteNumber(cfg.duration) && cfg.duration > 0) {
    const steps = Math.floor(cfg.duration / cfg.dt);
    if (!Number.isFinite(steps) || steps < 1) issues.push('duration/dt must produce at least one simulation step');
    if (steps > MAX_SIMULATION_STEPS) {
      issues.push(`duration/dt exceeds execution budget of ${MAX_SIMULATION_STEPS} steps`);
    }
  }

  for (const [group, fields] of [
    ['plant', ['stiffness', 'gain', 'damping']],
    ['pid', ['kp', 'ki', 'kd', 'uMin', 'uMax', 'antiWindup']],
  ]) {
    if (!isObject(cfg[group])) {
      issues.push(`${group} must be an object`);
      continue;
    }
    for (const field of fields) validateNumber(issues, `${group}.${field}`, cfg[group][field]);
  }
  if (isObject(cfg.pid)) {
    validateOrdered(issues, 'pid.uMin', cfg.pid.uMin, 'pid.uMax', cfg.pid.uMax);
    if (isFiniteNumber(cfg.pid.antiWindup) && cfg.pid.antiWindup < 0) issues.push('pid.antiWindup must be >= 0');
  }

  if (!isObject(cfg.truthPlant)) {
    issues.push('truthPlant must be an object');
  } else {
    validateBoolean(issues, 'truthPlant.enabled', cfg.truthPlant.enabled);
    for (const field of ['stiffnessScale', 'dampingScale', 'gainScale']) {
      validateNumber(issues, `truthPlant.${field}`, cfg.truthPlant[field], { min: 0, strictMin: true });
    }
  }

  if (!isObject(cfg.mpc)) {
    issues.push('mpc must be an object');
  } else {
    const solverValues = new Set(Object.values(SOLVER_BACKENDS));
    if (!solverValues.has(cfg.mpc.solver)) issues.push(`mpc.solver is unsupported: ${cfg.mpc.solver}`);
    validateNumber(issues, 'mpc.horizon', cfg.mpc.horizon, { min: 1, integer: true });
    for (const field of ['qPosition', 'qVelocity', 'rInput', 'rDelta', 'terminalWeight']) {
      validateNumber(issues, `mpc.${field}`, cfg.mpc[field], { min: 0 });
    }
    for (const field of ['iterations', 'qpIterations', 'qpProjectionCycles']) {
      validateNumber(issues, `mpc.${field}`, cfg.mpc[field], { min: 1, integer: true });
    }
    validateNumber(issues, 'mpc.learningRate', cfg.mpc.learningRate, { min: 0, strictMin: true });
    validateNumber(issues, 'mpc.qpTolerance', cfg.mpc.qpTolerance, { min: 0 });
    validateNumber(issues, 'mpc.qpStepScale', cfg.mpc.qpStepScale, { min: 0, strictMin: true });
    validateNumber(issues, 'mpc.qpProjectionTolerance', cfg.mpc.qpProjectionTolerance, { min: 0 });
    validateNumber(issues, 'mpc.qpFeasibilityTolerance', cfg.mpc.qpFeasibilityTolerance, { min: 0 });
    validateNumber(issues, 'mpc.qpTimeBudgetMs', cfg.mpc.qpTimeBudgetMs, { min: 0 });

    validateNumber(issues, 'mpc.uMin', cfg.mpc.uMin);
    validateNumber(issues, 'mpc.uMax', cfg.mpc.uMax);
    validateOrdered(issues, 'mpc.uMin', cfg.mpc.uMin, 'mpc.uMax', cfg.mpc.uMax);

    validateNumber(issues, 'mpc.deltaUMin', cfg.mpc.deltaUMin, { extended: true });
    validateNumber(issues, 'mpc.deltaUMax', cfg.mpc.deltaUMax, { extended: true });
    validateOrdered(issues, 'mpc.deltaUMin', cfg.mpc.deltaUMin, 'mpc.deltaUMax', cfg.mpc.deltaUMax, { extended: true });

    validateBoolean(issues, 'mpc.stateConstraintsEnabled', cfg.mpc.stateConstraintsEnabled);
    validateBoolean(issues, 'mpc.outputConstraintsEnabled', cfg.mpc.outputConstraintsEnabled);
    if (cfg.mpc.stateConstraintsEnabled === true) {
      for (const field of ['positionMin', 'positionMax', 'velocityMin', 'velocityMax']) {
        validateNumber(issues, `mpc.${field}`, cfg.mpc[field]);
      }
      validateOrdered(issues, 'mpc.positionMin', cfg.mpc.positionMin, 'mpc.positionMax', cfg.mpc.positionMax);
      validateOrdered(issues, 'mpc.velocityMin', cfg.mpc.velocityMin, 'mpc.velocityMax', cfg.mpc.velocityMax);
    }
    if (cfg.mpc.outputConstraintsEnabled === true) {
      validateNumber(issues, 'mpc.outputMin', cfg.mpc.outputMin);
      validateNumber(issues, 'mpc.outputMax', cfg.mpc.outputMax);
      validateOrdered(issues, 'mpc.outputMin', cfg.mpc.outputMin, 'mpc.outputMax', cfg.mpc.outputMax);
    }
    validateNumber(issues, 'mpc.referenceLead', cfg.mpc.referenceLead);
    validateNumber(issues, 'mpc.velocityDamping', cfg.mpc.velocityDamping, { min: 0 });
    validateNumber(issues, 'mpc.maxReferenceLead', cfg.mpc.maxReferenceLead, { min: 0 });
  }

  if (!isObject(cfg.safety)) {
    issues.push('safety must be an object');
  } else {
    validateNumber(issues, 'safety.previewHorizon', cfg.safety.previewHorizon, { min: 1, integer: true });
    for (const field of ['positionMargin', 'velocityMargin', 'outputMargin']) {
      validateNumber(issues, `safety.${field}`, cfg.safety[field], { min: 0 });
    }
  }

  if (!isObject(cfg.estimation)) {
    issues.push('estimation must be an object');
  } else {
    for (const field of [
      'enabled',
      'constraintTighteningEnabled',
      'disturbanceStateEnabled',
      'disturbancePredictionEnabled',
      'mpcDisturbanceCompensationEnabled',
    ]) validateBoolean(issues, `estimation.${field}`, cfg.estimation[field]);

    validateNumber(issues, 'estimation.measurementNoiseStd', cfg.estimation.measurementNoiseStd, { min: 0 });
    validateNumber(issues, 'estimation.measurementBias', cfg.estimation.measurementBias);
    validateNumber(issues, 'estimation.seed', cfg.estimation.seed, { min: 1, integer: true });
    for (const field of [
      'processPositionVariance',
      'processVelocityVariance',
      'initialPositionVariance',
      'initialVelocityVariance',
      'constraintSigma',
      'disturbanceProcessVariance',
      'initialDisturbanceVariance',
    ]) validateNumber(issues, `estimation.${field}`, cfg.estimation[field], { min: 0 });
    validateNumber(issues, 'estimation.disturbanceRetention', cfg.estimation.disturbanceRetention, { min: 0, max: 1 });
  }

  if (!isObject(cfg.trigger)) {
    issues.push('trigger must be an object');
  } else {
    for (const field of ['predictionError', 'stateChange', 'minInterval', 'maxInterval']) {
      validateNumber(issues, `trigger.${field}`, cfg.trigger[field], { min: 0 });
    }
    validateOrdered(issues, 'trigger.minInterval', cfg.trigger.minInterval, 'trigger.maxInterval', cfg.trigger.maxInterval);
    validateNumber(issues, 'trigger.constraintRatio', cfg.trigger.constraintRatio, { min: 0, max: 1 });
    validateNumber(issues, 'trigger.positionScale', cfg.trigger.positionScale, { min: 0, strictMin: true });
    validateNumber(issues, 'trigger.velocityScale', cfg.trigger.velocityScale, { min: 0, strictMin: true });
  }

  if (!isObject(cfg.disturbance)) {
    issues.push('disturbance must be an object');
  } else {
    validateBoolean(issues, 'disturbance.enabled', cfg.disturbance.enabled);
    validateNumber(issues, 'disturbance.start', cfg.disturbance.start, { min: 0 });
    validateNumber(issues, 'disturbance.duration', cfg.disturbance.duration, { min: 0 });
    validateNumber(issues, 'disturbance.amplitude', cfg.disturbance.amplitude);
  }

  return issues;
}

export function assertValidSimulationConfig(cfg) {
  const issues = validateSimulationConfig(cfg);
  if (issues.length) throw new SimulationConfigValidationError(issues);
  return cfg;
}
