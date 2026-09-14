const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export function createPIDController(params, dt) {
  let integral = 0;
  let prevError = 0;

  function calculate(target, value, dynamicLimits = null) {
    const e = target - value;
    const derivative = (e - prevError) / dt;
    const rawIntegral = integral + e * dt;
    const raw = params.kp * e + params.ki * rawIntegral + params.kd * derivative;

    let lower = params.uMin;
    let upper = params.uMax;
    if (dynamicLimits?.feasible !== false) {
      if (Number.isFinite(dynamicLimits?.lower)) lower = Math.max(lower, dynamicLimits.lower);
      if (Number.isFinite(dynamicLimits?.upper)) upper = Math.min(upper, dynamicLimits.upper);
    }

    const validLimits = lower <= upper;
    const u = validLimits ? clamp(raw, lower, upper) : clamp(raw, params.uMin, params.uMax);
    const saturated = Math.abs(raw - u) > 1e-10;
    integral = saturated ? integral + params.antiWindup * e * dt : rawIntegral;
    prevError = e;

    return {
      u,
      raw,
      error: e,
      derivative,
      saturated,
      lower,
      upper,
      dynamicLimited: saturated && (lower > params.uMin || upper < params.uMax),
      validLimits,
    };
  }

  return {
    update(target, value, dynamicLimits = null) {
      return calculate(target, value, dynamicLimits).u;
    },
    updateDetailed(target, value, dynamicLimits = null) {
      return calculate(target, value, dynamicLimits);
    },
    reset() {
      integral = 0;
      prevError = 0;
    },
  };
}
