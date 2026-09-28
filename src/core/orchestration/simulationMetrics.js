const average = (values) => values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0;
const rms = (values) => values.length
  ? Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length)
  : null;


function settlingTime(samples, target, tolerance = 0.02) {
  for (let i = 0; i < samples.length; i += 1) {
    if (samples.slice(i).every((p) => Math.abs(p.x - target) <= tolerance)) return samples[i].t;
  }
  return null;
}

export function safetyViolationAt(sample, cfg) {
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

export function computeSimulationMetrics(samples, target, solverRecords, cfg) {
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

  const governorSamples = samples.filter((sample) => sample.governorEnabled);
  const safetyInterventions = governorSamples.filter((sample) => sample.governorSafetyIntervened);
  const conditioningInterventions = governorSamples.filter((sample) => sample.governorConditioned);
  const safetyCorrections = governorSamples.map((sample) => Math.abs(sample.governorCorrection || 0));
  const conditioningCorrections = governorSamples.map((sample) => Math.abs(sample.governorConditioningCorrection || 0));
  const governorWidths = governorSamples.map((sample) => sample.governorIntervalWidth).filter(Number.isFinite);
  const governorInfeasibleCount = governorSamples.filter((sample) => sample.governorFeasible === false).length;
  const governorEmergencyCount = governorSamples.filter((sample) => sample.governorEmergencyFallback).length;
  const governorContinuationCount = governorSamples.filter((sample) => sample.governorContinuationUsed).length;

  const estimationSamples = samples.filter((sample) => sample.estimationEnabled);
  const measurementErrors = estimationSamples
    .map((sample) => sample.measurement - sample.measurementTruth)
    .filter(Number.isFinite);
  const estimatePositionErrors = estimationSamples
    .map((sample) => sample.estimateX - sample.x)
    .filter(Number.isFinite);
  const estimateVelocityErrors = estimationSamples
    .map((sample) => sample.estimateV - sample.v)
    .filter(Number.isFinite);
  const innovations = estimationSamples.map((sample) => sample.innovation).filter(Number.isFinite);
  const covarianceTraces = estimationSamples.map((sample) => sample.covarianceTrace).filter(Number.isFinite);
  const measurementRmse = rms(measurementErrors);
  const estimatePositionRmse = rms(estimatePositionErrors);

  const disturbanceEstimateSamples = samples.filter((sample) => sample.disturbanceEstimateEnabled);
  const disturbanceEstimateErrors = disturbanceEstimateSamples
    .map((sample) => sample.estimateD - sample.equivalentDisturbance)
    .filter(Number.isFinite);
  const activeDisturbanceEstimateErrors = disturbanceEstimateSamples
    .filter((sample) => Math.abs(sample.equivalentDisturbance) > 0.05)
    .map((sample) => sample.estimateD - sample.equivalentDisturbance)
    .filter(Number.isFinite);
  const disturbanceVariances = disturbanceEstimateSamples
    .map((sample) => sample.disturbanceVariance)
    .filter(Number.isFinite);
  const predictionCompensatedSamples = samples.filter((sample) => sample.disturbancePredictionEnabled);
  const mpcDisturbanceCompensatedSamples = samples.filter((sample) => sample.mpcDisturbanceCompensationEnabled);

  const tighteningSamples = samples.filter((sample) => sample.uncertaintyTighteningEnabled);
  const positionMargins = tighteningSamples.map((sample) => sample.uncertaintyPositionMargin).filter(Number.isFinite);
  const velocityMargins = tighteningSamples.map((sample) => sample.uncertaintyVelocityMargin).filter(Number.isFinite);
  const outputMargins = tighteningSamples.map((sample) => sample.uncertaintyOutputMargin).filter(Number.isFinite);
  const tighteningInvalidCount = tighteningSamples.filter((sample) => sample.uncertaintyEnvelopeValid === false).length;

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
    governorEnabled: governorSamples.length > 0,
    governorInterventionCount: safetyInterventions.length,
    governorInterventionRate: governorSamples.length ? 100 * safetyInterventions.length / governorSamples.length : 0,
    governorAvgCorrection: average(safetyCorrections),
    governorMaxCorrection: safetyCorrections.length ? Math.max(...safetyCorrections) : 0,
    governorConditioningCount: conditioningInterventions.length,
    governorConditioningRate: governorSamples.length ? 100 * conditioningInterventions.length / governorSamples.length : 0,
    governorAvgConditioningCorrection: average(conditioningCorrections),
    governorMaxConditioningCorrection: conditioningCorrections.length ? Math.max(...conditioningCorrections) : 0,
    governorAvgIntervalWidth: average(governorWidths),
    governorInfeasibleCount,
    governorEmergencyCount,
    governorContinuationRate: governorSamples.length ? 100 * governorContinuationCount / governorSamples.length : 0,
    estimationEnabled: estimationSamples.length > 0,
    measurementRmse,
    estimatePositionRmse,
    estimateVelocityRmse: rms(estimateVelocityErrors),
    innovationRmse: rms(innovations),
    avgCovarianceTrace: average(covarianceTraces),
    finalCovarianceTrace: covarianceTraces.length ? covarianceTraces[covarianceTraces.length - 1] : null,
    estimateToMeasurementRmseRatio: measurementRmse && estimatePositionRmse != null
      ? estimatePositionRmse / measurementRmse
      : null,
    disturbanceEstimateEnabled: disturbanceEstimateSamples.length > 0,
    disturbanceEstimateRmse: rms(disturbanceEstimateErrors),
    activeDisturbanceEstimateRmse: rms(activeDisturbanceEstimateErrors),
    avgDisturbanceVariance: average(disturbanceVariances),
    finalDisturbanceVariance: disturbanceVariances.length ? disturbanceVariances[disturbanceVariances.length - 1] : null,
    disturbancePredictionEnabled: predictionCompensatedSamples.length > 0,
    mpcDisturbanceCompensationEnabled: mpcDisturbanceCompensatedSamples.length > 0,
    truthPlantMismatchEnabled: samples.some((sample) => sample.truthPlantMismatchEnabled),
    uncertaintyTighteningEnabled: tighteningSamples.length > 0,
    avgUncertaintyPositionMargin: average(positionMargins),
    maxUncertaintyPositionMargin: positionMargins.length ? Math.max(...positionMargins) : 0,
    avgUncertaintyVelocityMargin: average(velocityMargins),
    maxUncertaintyVelocityMargin: velocityMargins.length ? Math.max(...velocityMargins) : 0,
    avgUncertaintyOutputMargin: average(outputMargins),
    maxUncertaintyOutputMargin: outputMargins.length ? Math.max(...outputMargins) : 0,
    uncertaintyInvalidEnvelopeCount: tighteningInvalidCount,
  };
}
