export function normalizedDistance(a, b, trigger) {
  const dx = (a.x - b.x) / Math.max(trigger.positionScale, 1e-9);
  const dv = (a.v - b.v) / Math.max(trigger.velocityScale, 1e-9);
  return Math.sqrt(dx * dx + dv * dv);
}

export function evaluateEventTrigger({
  k,
  t,
  state,
  expectedState,
  lastSolveState,
  previousU,
  lastSolve,
  cfg,
}) {
  const predictionError = k === 0 ? 0 : normalizedDistance(state, expectedState, cfg.trigger);
  const stateChange = normalizedDistance(state, lastSolveState, cfg.trigger);
  const elapsed = t - lastSolve;
  const constraintRatio = Math.abs(previousU) / Math.max(Math.abs(cfg.pid.uMax), 1e-9);
  const intervalReady = elapsed >= cfg.trigger.minInterval;
  const watchdog = elapsed >= cfg.trigger.maxInterval;
  const predictionEvent = intervalReady && predictionError >= cfg.trigger.predictionError;
  const stateEvent = intervalReady && stateChange >= cfg.trigger.stateChange;
  const constraintEvent = intervalReady && constraintRatio >= cfg.trigger.constraintRatio;
  const triggered = k === 0 || watchdog || predictionEvent || stateEvent || constraintEvent;

  const reason = !triggered
    ? ''
    : k === 0
      ? 'initial'
      : watchdog
        ? 'watchdog'
        : predictionEvent
          ? 'prediction-error'
          : constraintEvent
            ? 'constraint'
            : 'state-change';

  return {
    triggered,
    reason,
    predictionError,
    stateChange,
    elapsed,
    constraintRatio,
  };
}
