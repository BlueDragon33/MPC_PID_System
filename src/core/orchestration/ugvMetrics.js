const rms = (values) => Math.sqrt(
  values.reduce((sum, value) => sum + value * value, 0) / Math.max(1, values.length),
);

const average = (values) => (
  values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0
);

export function computeUgvMetrics(samples, solverRecords, cfg) {
  const lateral = samples.map((sample) => sample.lateralError);
  const heading = samples.map((sample) => sample.headingError);
  const speed = samples.map((sample) => sample.speedError);
  const safety = samples.map((sample) => sample.safetyViolation);
  const estimateX = samples.filter((sample) => sample.estimationEnabled).map((sample) => sample.estimateX - sample.x);
  const estimateY = samples.filter((sample) => sample.estimationEnabled).map((sample) => sample.estimateY - sample.y);
  const estimateYaw = samples.filter((sample) => sample.estimationEnabled).map((sample) => sample.estimateYawError);
  const estimateV = samples.filter((sample) => sample.estimationEnabled).map((sample) => sample.estimateV - sample.v);

  const solved = solverRecords.filter((record) => record.status === 'solved').length;
  const accepted = solverRecords.filter((record) => record.status === 'solved' || record.status === 'max-iterations').length;
  const fallback = solverRecords.filter((record) => record.fallbackUsed).length;
  const solveTimes = solverRecords.map((record) => record.solveMs).filter(Number.isFinite);

  return {
    duration: cfg.duration,
    samples: samples.length,
    lateralRmse: rms(lateral),
    maxAbsLateralError: Math.max(...lateral.map(Math.abs)),
    headingRmse: rms(heading),
    maxAbsHeadingError: Math.max(...heading.map(Math.abs)),
    speedRmse: rms(speed),
    controlEffort: samples.reduce(
      (sum, sample) => sum + cfg.dt * (
        sample.acceleration * sample.acceleration
        + sample.steering * sample.steering
      ),
      0,
    ),
    maxSafetyViolation: Math.max(...safety),
    unsafeSamples: safety.filter((value) => value > 1e-12).length,
    unsafeRate: 100 * safety.filter((value) => value > 1e-12).length / Math.max(1, samples.length),
    solveCount: solverRecords.length,
    solvedCount: solved,
    acceptedSolveCount: accepted,
    convergenceRate: solverRecords.length ? 100 * solved / solverRecords.length : null,
    acceptedRate: solverRecords.length ? 100 * accepted / solverRecords.length : null,
    fallbackCount: fallback,
    averageSolveMs: average(solveTimes),
    maxSolveMs: solveTimes.length ? Math.max(...solveTimes) : 0,
    estimateXRmse: estimateX.length ? rms(estimateX) : null,
    estimateYRmse: estimateY.length ? rms(estimateY) : null,
    estimateYawRmse: estimateYaw.length ? rms(estimateYaw) : null,
    estimateSpeedRmse: estimateV.length ? rms(estimateV) : null,
    finalProgressX: samples.at(-1)?.x ?? 0,
  };
}
