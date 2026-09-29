export const CONTROL_MODES = ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'];

export const RESPONSE_SIGNALS = [
  { value: 'x', label: 'Position (x)', yLabel: 'Position' },
  { value: 'v', label: 'Velocity (v)', yLabel: 'Velocity' },
];

export function normalizeControlMode(value) {
  return CONTROL_MODES.includes(value) ? value : 'HYBRID_SAFE';
}

export function normalizeResponseSignal(value) {
  return RESPONSE_SIGNALS.some((signal) => signal.value === value) ? value : 'x';
}

export function buildSimulationView(results, activeMode) {
  if (!Array.isArray(results) || results.length === 0) {
    return { active: null, visibleResults: [] };
  }
  const normalizedMode = normalizeControlMode(activeMode);
  const active = results.find((result) => result.mode === normalizedMode)
    || results.find((result) => result.mode === 'HYBRID_SAFE')
    || results[0];
  return { active, visibleResults: [active] };
}
