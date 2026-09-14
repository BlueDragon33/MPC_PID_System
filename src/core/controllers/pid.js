const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export function createPIDController(params, dt) {
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
    reset() {
      integral = 0;
      prevError = 0;
    },
  };
}
