const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

export function normalizeAngle(angle) {
  let result = angle;
  while (result > Math.PI) result -= 2 * Math.PI;
  while (result < -Math.PI) result += 2 * Math.PI;
  return result;
}

export function createUgvInitialState(overrides = {}) {
  return {
    x: Number.isFinite(overrides.x) ? overrides.x : 0,
    y: Number.isFinite(overrides.y) ? overrides.y : 0.6,
    yaw: Number.isFinite(overrides.yaw) ? overrides.yaw : 0,
    v: Number.isFinite(overrides.v) ? overrides.v : 0,
  };
}

export function saturateUgvCommand(command, previousCommand, cfg) {
  const limits = cfg.ugv;
  const dt = cfg.dt;
  const previous = previousCommand ?? { acceleration: 0, steering: 0 };

  const accelerationByMagnitude = clamp(
    Number.isFinite(command.acceleration) ? command.acceleration : 0,
    limits.accelerationMin,
    limits.accelerationMax,
  );
  const steeringByMagnitude = clamp(
    Number.isFinite(command.steering) ? command.steering : 0,
    limits.steeringMin,
    limits.steeringMax,
  );

  const acceleration = clamp(
    accelerationByMagnitude,
    previous.acceleration + limits.accelerationRateMin * dt,
    previous.acceleration + limits.accelerationRateMax * dt,
  );
  const steering = clamp(
    steeringByMagnitude,
    previous.steering + limits.steeringRateMin * dt,
    previous.steering + limits.steeringRateMax * dt,
  );

  return {
    acceleration: clamp(acceleration, limits.accelerationMin, limits.accelerationMax),
    steering: clamp(steering, limits.steeringMin, limits.steeringMax),
  };
}

export function kinematicBicycleDerivative(state, command, cfg) {
  const wheelbase = Math.max(1e-6, cfg.ugv.wheelbase);
  const steering = clamp(command.steering, cfg.ugv.steeringMin, cfg.ugv.steeringMax);
  const acceleration = clamp(command.acceleration, cfg.ugv.accelerationMin, cfg.ugv.accelerationMax);
  const v = clamp(state.v, cfg.ugv.speedMin, cfg.ugv.speedMax);

  return {
    x: v * Math.cos(state.yaw),
    y: v * Math.sin(state.yaw),
    yaw: (v / wheelbase) * Math.tan(steering),
    v: acceleration,
  };
}

function addScaled(state, derivative, scale) {
  return {
    x: state.x + derivative.x * scale,
    y: state.y + derivative.y * scale,
    yaw: state.yaw + derivative.yaw * scale,
    v: state.v + derivative.v * scale,
  };
}

export function stepKinematicBicycle(state, command, cfg) {
  const dt = cfg.dt;
  const k1 = kinematicBicycleDerivative(state, command, cfg);
  const k2 = kinematicBicycleDerivative(addScaled(state, k1, dt / 2), command, cfg);
  const k3 = kinematicBicycleDerivative(addScaled(state, k2, dt / 2), command, cfg);
  const k4 = kinematicBicycleDerivative(addScaled(state, k3, dt), command, cfg);

  return {
    x: state.x + (dt / 6) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
    y: state.y + (dt / 6) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
    yaw: normalizeAngle(state.yaw + (dt / 6) * (k1.yaw + 2 * k2.yaw + 2 * k3.yaw + k4.yaw)),
    v: clamp(
      state.v + (dt / 6) * (k1.v + 2 * k2.v + 2 * k3.v + k4.v),
      cfg.ugv.speedMin,
      cfg.ugv.speedMax,
    ),
  };
}

export function linearizeKinematicBicycle(state, command, cfg) {
  const dt = cfg.dt;
  const wheelbase = Math.max(1e-6, cfg.ugv.wheelbase);
  const steering = clamp(command.steering, cfg.ugv.steeringMin, cfg.ugv.steeringMax);
  const sec2 = 1 / (Math.cos(steering) ** 2);
  const v = state.v;
  const yaw = state.yaw;

  // First-order discrete Jacobian of Euler-discretized bicycle dynamics.
  const A = [
    [1, 0, -dt * v * Math.sin(yaw), dt * Math.cos(yaw)],
    [0, 1, dt * v * Math.cos(yaw), dt * Math.sin(yaw)],
    [0, 0, 1, dt * Math.tan(steering) / wheelbase],
    [0, 0, 0, 1],
  ];
  const B = [
    [0, 0],
    [0, 0],
    [0, dt * v * sec2 / wheelbase],
    [dt, 0],
  ];

  return { A, B };
}
