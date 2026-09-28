import { normalizeAngle } from './kinematicBicycle.js';

export function ugvReferenceAtX(x, cfg) {
  const amplitude = cfg.ugvPath.amplitude;
  const waveNumber = cfg.ugvPath.waveNumber;
  const y = amplitude * Math.sin(waveNumber * x);
  const dy = amplitude * waveNumber * Math.cos(waveNumber * x);
  const ddy = -amplitude * waveNumber * waveNumber * Math.sin(waveNumber * x);
  const yaw = Math.atan(dy);
  const curvature = ddy / Math.pow(1 + dy * dy, 1.5);
  const speed = cfg.ugvPath.referenceSpeed;
  return { x, y, yaw, curvature, speed };
}

export function ugvTrackingErrors(state, reference) {
  const dx = state.x - reference.x;
  const dy = state.y - reference.y;
  const lateral = -Math.sin(reference.yaw) * dx + Math.cos(reference.yaw) * dy;
  return {
    lateral,
    heading: normalizeAngle(state.yaw - reference.yaw),
    speed: state.v - reference.speed,
  };
}

export function ugvSafetyViolation(state, cfg) {
  const reference = ugvReferenceAtX(state.x, cfg);
  const error = ugvTrackingErrors(state, reference);
  const laneViolation = Math.max(0, Math.abs(error.lateral) - cfg.ugvSafety.laneHalfWidth);
  const speedViolation = Math.max(
    0,
    cfg.ugvSafety.speedMin - state.v,
    state.v - cfg.ugvSafety.speedMax,
  );
  return {
    total: Math.max(laneViolation, speedViolation),
    lane: laneViolation,
    speed: speedViolation,
    error,
    reference,
  };
}
