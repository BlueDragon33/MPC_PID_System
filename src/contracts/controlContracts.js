export class ControlContractError extends Error {
  constructor(message) {
    super(message);
    this.name = 'ControlContractError';
  }
}

export function assertFiniteNumber(value, label) {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new ControlContractError(`${label} must be finite.`);
  }
  return value;
}

export function assertPositiveNumber(value, label) {
  assertFiniteNumber(value, label);
  if (!(value > 0)) throw new ControlContractError(`${label} must be > 0.`);
  return value;
}

export function assertNonnegativeNumber(value, label) {
  assertFiniteNumber(value, label);
  if (value < 0) throw new ControlContractError(`${label} must be >= 0.`);
  return value;
}

export function assertOrderedBounds(lower, upper, lowerLabel, upperLabel) {
  assertFiniteNumber(lower, lowerLabel);
  assertFiniteNumber(upper, upperLabel);
  if (lower > upper) {
    throw new ControlContractError(`${lowerLabel} must be <= ${upperLabel}.`);
  }
}

export function assertFiniteRecord(value, fields, label) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    throw new ControlContractError(`${label} must be an object.`);
  }
  for (const field of fields) assertFiniteNumber(value[field], `${label}.${field}`);
  return value;
}

export function assertFiniteVector(value, length, label) {
  if (!Array.isArray(value) || value.length !== length) {
    throw new ControlContractError(`${label} must contain exactly ${length} values.`);
  }
  value.forEach((item, index) => assertFiniteNumber(item, `${label}[${index}]`));
  return value;
}

export function assertNonnegativeVector(value, length, label) {
  assertFiniteVector(value, length, label);
  value.forEach((item, index) => {
    if (item < 0) throw new ControlContractError(`${label}[${index}] must be >= 0.`);
  });
  return value;
}

export function assertBicycleConfigContract(cfg) {
  assertPositiveNumber(cfg.dt, 'bicycle.dt');
  assertPositiveNumber(cfg.wheelbase, 'bicycle.wheelbase');
  assertNonnegativeNumber(cfg.maxSteer, 'bicycle.maxSteer');
  assertNonnegativeNumber(cfg.maxSteerRate, 'bicycle.maxSteerRate');
  assertOrderedBounds(cfg.minSpeed, cfg.maxSpeed, 'bicycle.minSpeed', 'bicycle.maxSpeed');
  assertOrderedBounds(cfg.minAccel, cfg.maxAccel, 'bicycle.minAccel', 'bicycle.maxAccel');
  return cfg;
}

export function assertPlanarUavConfigContract(cfg) {
  assertPositiveNumber(cfg.dt, 'uav.dt');
  assertPositiveNumber(cfg.mass, 'uav.mass');
  assertPositiveNumber(cfg.inertia, 'uav.inertia');
  assertPositiveNumber(cfg.gravity, 'uav.gravity');
  assertNonnegativeNumber(cfg.linearDragX, 'uav.linearDragX');
  assertNonnegativeNumber(cfg.linearDragZ, 'uav.linearDragZ');
  assertNonnegativeNumber(cfg.angularDamping, 'uav.angularDamping');
  assertOrderedBounds(cfg.minThrust, cfg.maxThrust, 'uav.minThrust', 'uav.maxThrust');
  assertNonnegativeNumber(cfg.maxTorque, 'uav.maxTorque');
  return cfg;
}

export function assertPlanarUsvConfigContract(cfg) {
  assertPositiveNumber(cfg.dt, 'usv.dt');
  assertPositiveNumber(cfg.mass, 'usv.mass');
  assertPositiveNumber(cfg.yawInertia, 'usv.yawInertia');
  for (const field of [
    'surgeLinearDrag',
    'surgeQuadraticDrag',
    'swayLinearDrag',
    'swayQuadraticDrag',
    'yawLinearDrag',
    'yawQuadraticDrag',
  ]) assertNonnegativeNumber(cfg[field], `usv.${field}`);
  assertOrderedBounds(cfg.minSurgeForce, cfg.maxSurgeForce, 'usv.minSurgeForce', 'usv.maxSurgeForce');
  assertNonnegativeNumber(cfg.maxYawMoment, 'usv.maxYawMoment');
  return cfg;
}

export function assertPidContract(params, dt) {
  assertPositiveNumber(dt, 'pid.dt');
  if (!params || typeof params !== 'object' || Array.isArray(params)) {
    throw new ControlContractError('pid.params must be an object.');
  }
  for (const field of ['kp','ki','kd','antiWindup']) {
    assertNonnegativeNumber(params[field], `pid.${field}`);
  }
  assertOrderedBounds(params.uMin, params.uMax, 'pid.uMin', 'pid.uMax');
  return params;
}

export function assertEstimatorContract({
  stateLength,
  initialState,
  initialCovariance,
  processCovariance,
  measurementVariance,
  label,
}) {
  assertFiniteVector(initialState, stateLength, `${label}.initialState`);
  assertNonnegativeVector(initialCovariance, stateLength, `${label}.initialCovariance`);
  assertNonnegativeVector(processCovariance, stateLength, `${label}.processCovariance`);
  assertNonnegativeVector(measurementVariance, stateLength, `${label}.measurementVariance`);
  return true;
}

export function assertSolverResultContract(result, expectedHorizon = null) {
  if (!result || typeof result !== 'object') throw new ControlContractError('solver result must be an object.');
  assertFiniteNumber(result.u, 'solver.u');
  if (!Array.isArray(result.sequence) || result.sequence.length === 0) {
    throw new ControlContractError('solver.sequence must be a non-empty array.');
  }
  result.sequence.forEach((value,index)=>assertFiniteNumber(value,`solver.sequence[${index}]`));
  if (expectedHorizon != null && result.sequence.length !== expectedHorizon) {
    throw new ControlContractError(`solver.sequence must match horizon ${expectedHorizon}.`);
  }
  if (typeof result.solver !== 'string' || !result.solver) throw new ControlContractError('solver.solver must identify the backend.');
  if (result.fallbackUsed === false && result.diagnostics?.finite === false) {
    throw new ControlContractError('non-fallback solver result cannot report non-finite diagnostics.');
  }
  if (result.status === 'solved' && result.diagnostics?.feasibilityViolation > 1e-6) {
    throw new ControlContractError('solved result cannot violate hard feasibility tolerance.');
  }
  return result;
}

export function assertSafetyGovernorResultContract(result) {
  if (!result || typeof result !== 'object') throw new ControlContractError('safety result must be an object.');
  assertFiniteNumber(result.u, 'safety.u');
  assertFiniteNumber(result.proposedU, 'safety.proposedU');
  assertFiniteNumber(result.correction, 'safety.correction');
  if (typeof result.feasible !== 'boolean') throw new ControlContractError('safety.feasible must be boolean.');
  if (typeof result.emergencyFallback !== 'boolean') throw new ControlContractError('safety.emergencyFallback must be boolean.');
  if (!result.interval || typeof result.interval !== 'object') throw new ControlContractError('safety.interval must be present.');
  return result;
}
