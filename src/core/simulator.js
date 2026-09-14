import { createPIDController } from './controllers/pid.js';
import { createAugmentedDisturbanceKalmanFilter } from './estimation/augmentedDisturbanceKalmanFilter.js';
import { createLinearKalmanFilter } from './estimation/linearKalmanFilter.js';
import { createMeasurementSensor } from './estimation/measurementSensor.js';
import { applyCovarianceConstraintTightening } from './estimation/uncertaintyTightening.js';
import {
  createSecondOrderModel,
  createTruthPlantConfig,
  disturbanceAt,
  equivalentDisturbance,
  stepSecondOrderPlant,
} from './models/secondOrderPlant.js';
import {
  computeAdmissibleCommandInterval,
  computePhysicalCommandInterval,
  applySafetyGovernor,
} from './safety/shortHorizonGovernor.js';
import { solveMPC, SOLVER_BACKENDS } from './solvers/index.js';
import { evaluateEventTrigger } from './triggers/eventTrigger.js';

export const defaultConfig = {
  dt: 0.02,
  duration: 12,
  setpoint: 1,
  plant: { stiffness: 1.45, gain: 1.0, damping: 0.82 },
  truthPlant: {
    enabled: false,
    stiffnessScale: 1,
    dampingScale: 1,
    gainScale: 1,
  },
  pid: { kp: 5.2, ki: 1.35, kd: 0.52, uMin: -4, uMax: 4, antiWindup: 0.5 },
  mpc: {
    solver: SOLVER_BACKENDS.CONSTRAINED_QP,
    horizon: 35,
    qPosition: 9,
    qVelocity: 1.2,
    rInput: 0.16,
    rDelta: 0.12,
    terminalWeight: 8,
    iterations: 20,
    learningRate: 0.12,
    qpIterations: 60,
    qpTolerance: 5e-4,
    qpStepScale: 0.95,
    qpProjectionCycles: 6,
    qpProjectionTolerance: 1e-9,
    qpFeasibilityTolerance: 1e-7,
    qpTimeBudgetMs: 0,
    uMin: -4,
    uMax: 4,
    deltaUMin: -0.65,
    deltaUMax: 0.65,
    stateConstraintsEnabled: false,
    positionMin: -1.5,
    positionMax: 1.5,
    velocityMin: -1.2,
    velocityMax: 1.2,
    outputConstraintsEnabled: false,
    outputMin: -0.2,
    outputMax: 1.05,
    referenceLead: 0.5,
    velocityDamping: 0.08,
    maxReferenceLead: 0.4,
  },
  safety: {
    previewHorizon: 6,
    positionMargin: 0,
    velocityMargin: 0,
    outputMargin: 0,
  },
  estimation: {
    enabled: false,
    measurementNoiseStd: 0.08,
    measurementBias: 0,
    seed: 20260914,
    processPositionVariance: 2e-5,
    processVelocityVariance: 2e-4,
    initialPositionVariance: 0.25,
    initialVelocityVariance: 0.8,
    constraintTighteningEnabled: false,
    constraintSigma: 2.0,
    disturbanceStateEnabled: false,
    disturbanceProcessVariance: 8e-3,
    initialDisturbanceVariance: 0.8,
  },
  trigger: {
    predictionError: 0.035,
    stateChange: 0.08,
    minInterval: 0.08,
    maxInterval: 0.36,
    constraintRatio: 0.92,
    positionScale: 1,
    velocityScale: 1,
  },
  disturbance: { enabled: true, start: 4.0, duration: 1.0, amplitude: 1.6 },
};

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const average = (values) => values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0;
const rms = (values) => values.length
  ? Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length)
  : null;

export function getStateSpaceModel(cfg = defaultConfig) {
  return createSecondOrderModel(cfg);
}

function predictiveReference(solution, target, cfg) {
  const future = solution.path[solution.path.length - 1] ?? { x: target, v: 0 };
  const rawLead = cfg.mpc.referenceLead * (target - future.x) - cfg.mpc.velocityDamping * future.v;
  const lead = clamp(rawLead, -cfg.mpc.maxReferenceLead, cfg.mpc.maxReferenceLead);
  return target + lead;
}

function settlingTime(samples, target, tolerance = 0.02) {
  for (let i = 0; i < samples.length; i += 1) {
    if (samples.slice(i).every((p) => Math.abs(p.x - target) <= tolerance)) return samples[i].t;
  }
  return null;
}

function safetyViolationAt(sample, cfg) {
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

function metrics(samples, target, solverRecords, cfg) {
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

function mergeConfig(userConfig) {
  return {
    ...defaultConfig,
    ...userConfig,
    plant: { ...defaultConfig.plant, ...(userConfig.plant || {}) },
    truthPlant: { ...defaultConfig.truthPlant, ...(userConfig.truthPlant || {}) },
    pid: { ...defaultConfig.pid, ...(userConfig.pid || {}) },
    mpc: { ...defaultConfig.mpc, ...(userConfig.mpc || {}) },
    safety: { ...defaultConfig.safety, ...(userConfig.safety || {}) },
    estimation: { ...defaultConfig.estimation, ...(userConfig.estimation || {}) },
    trigger: { ...defaultConfig.trigger, ...(userConfig.trigger || {}) },
    disturbance: { ...defaultConfig.disturbance, ...(userConfig.disturbance || {}) },
  };
}

export function runSimulation(mode, userConfig = {}) {
  const cfg = mergeConfig(userConfig);
  const steps = Math.floor(cfg.duration / cfg.dt);
  const pid = createPIDController(cfg.pid, cfg.dt);
  const model = createSecondOrderModel(cfg);
  const truthCfg = createTruthPlantConfig(cfg);
  const truthModel = createSecondOrderModel(truthCfg);
  const estimationEnabled = Boolean(cfg.estimation.enabled);
  const disturbanceStateEnabled = Boolean(estimationEnabled && cfg.estimation.disturbanceStateEnabled);
  const sensor = createMeasurementSensor({
    C: model.C,
    noiseStd: estimationEnabled ? cfg.estimation.measurementNoiseStd : 0,
    seed: cfg.estimation.seed,
    bias: estimationEnabled ? cfg.estimation.measurementBias : 0,
  });

  let state = { x: 0, v: 0 };
  let controllerState = { ...state };
  let measurementSample = sensor.read(state);
  let estimator = null;
  let estimatorDiagnostics = null;
  let estimatorCovariance = null;
  let covarianceTrace = null;
  let disturbanceEstimate = 0;
  let disturbanceVariance = null;

  if (estimationEnabled) {
    estimator = disturbanceStateEnabled
      ? createAugmentedDisturbanceKalmanFilter({
          A: model.A,
          B: model.B,
          E: model.E,
          C: model.C,
          processCovariance: [
            cfg.estimation.processPositionVariance,
            cfg.estimation.processVelocityVariance,
            cfg.estimation.disturbanceProcessVariance,
          ],
          measurementVariance: Math.max(1e-12, cfg.estimation.measurementNoiseStd ** 2),
          initialState: [state.x, state.v, 0],
          initialCovariance: [
            cfg.estimation.initialPositionVariance,
            cfg.estimation.initialVelocityVariance,
            cfg.estimation.initialDisturbanceVariance,
          ],
        })
      : createLinearKalmanFilter({
          A: model.A,
          B: model.B,
          C: model.C,
          processCovariance: [
            cfg.estimation.processPositionVariance,
            cfg.estimation.processVelocityVariance,
          ],
          measurementVariance: Math.max(1e-12, cfg.estimation.measurementNoiseStd ** 2),
          initialState: [state.x, state.v],
          initialCovariance: [
            cfg.estimation.initialPositionVariance,
            cfg.estimation.initialVelocityVariance,
          ],
        });
    const initialEstimate = estimator.update(measurementSample.value);
    controllerState = { x: initialEstimate.x, v: initialEstimate.v };
    disturbanceEstimate = disturbanceStateEnabled ? initialEstimate.d : 0;
    estimatorDiagnostics = initialEstimate.diagnostics;
    estimatorCovariance = initialEstimate.covariance;
    covarianceTrace = estimatorDiagnostics.covarianceTrace;
    disturbanceVariance = disturbanceStateEnabled ? estimatorCovariance?.[2]?.[2] ?? null : null;
  }

  let expectedState = { ...controllerState };
  let lastSolveState = { ...controllerState };
  let previousU = 0;
  let lastMpcSafeU = 0;
  let lastMpcPlan = null;
  let lastSolve = -Infinity;
  let reference = cfg.setpoint;
  let warmStart = null;
  const solverRecords = [];
  const samples = [];
  const hybridMode = mode === 'HYBRID' || mode === 'HYBRID_SAFE';
  const governorMode = mode === 'HYBRID_SAFE';

  function recordSolution(solution) {
    solverRecords.push({
      solveMs: solution.solveMs,
      solver: solution.solver,
      status: solution.status || (solution.diagnostics?.converged ? 'solved' : null),
      fallbackUsed: Boolean(solution.fallbackUsed),
      fallbackReason: solution.fallbackReason || null,
      diagnostics: solution.diagnostics || null,
    });
  }

  function acceptMpcPlan(solution) {
    if (solution.fallbackUsed) return;
    lastMpcPlan = [...solution.sequence];
    lastMpcSafeU = solution.u;
  }

  function shiftMpcPlan() {
    if (!lastMpcPlan?.length) return;
    const tail = lastMpcPlan[lastMpcPlan.length - 1];
    lastMpcPlan = [...lastMpcPlan.slice(1), tail];
    if (Number.isFinite(lastMpcPlan[0])) lastMpcSafeU = lastMpcPlan[0];
  }

  for (let k = 0; k <= steps; k += 1) {
    const t = k * cfg.dt;
    const { config: controlCfg, tightening } = applyCovarianceConstraintTightening(
      cfg,
      estimatorCovariance,
      model.C,
    );
    const event = evaluateEventTrigger({
      k,
      t,
      state: controllerState,
      expectedState,
      lastSolveState,
      previousU,
      lastSolve,
      cfg: controlCfg,
    });

    let u = previousU;
    let triggered = false;
    let triggerReason = '';
    let mpcCost = null;
    let solverDiagnostics = null;
    let solverStatus = null;
    let fallbackUsed = false;
    let governor = null;
    let pidRaw = null;
    let pidNominal = null;
    let pidConditioned = null;

    if (mode === 'PID') {
      u = pid.update(controlCfg.setpoint, controllerState.x);
    } else if (mode === 'MPC') {
      const solution = solveMPC(controllerState, controlCfg.setpoint, previousU, controlCfg, warmStart);
      u = solution.u;
      warmStart = solution.sequence;
      acceptMpcPlan(solution);
      mpcCost = solution.cost;
      solverDiagnostics = solution.diagnostics || null;
      solverStatus = solution.status || null;
      fallbackUsed = Boolean(solution.fallbackUsed);
      recordSolution(solution);
      triggered = true;
      triggerReason = 'periodic';
    } else if (hybridMode) {
      if (event.triggered) {
        const solution = solveMPC(controllerState, controlCfg.setpoint, previousU, controlCfg, warmStart);
        warmStart = solution.sequence;
        if (!solution.fallbackUsed) {
          reference = predictiveReference(solution, controlCfg.setpoint, controlCfg);
          acceptMpcPlan(solution);
        }
        lastSolve = t;
        lastSolveState = { ...controllerState };
        mpcCost = solution.cost;
        solverDiagnostics = solution.diagnostics || null;
        solverStatus = solution.status || null;
        fallbackUsed = Boolean(solution.fallbackUsed);
        recordSolution(solution);
        triggered = true;
        triggerReason = event.reason;
      }

      if (governorMode) {
        const interval = computeAdmissibleCommandInterval(controllerState, previousU, controlCfg, lastMpcPlan);
        const baseInterval = interval.baseInterval ?? computePhysicalCommandInterval(previousU, controlCfg);
        const pidResult = pid.updateDetailed(reference, controllerState.x, interval.feasible ? interval : null);
        pidRaw = pidResult.raw;
        pidNominal = clamp(pidResult.raw, controlCfg.pid.uMin, controlCfg.pid.uMax);
        pidConditioned = baseInterval.feasible
          ? clamp(pidNominal, baseInterval.lower, baseInterval.upper)
          : pidNominal;
        const conditioningCorrection = pidConditioned - pidNominal;

        if (interval.feasible) {
          u = pidResult.u;
          const safetyCorrection = u - pidConditioned;
          governor = {
            feasible: true,
            emergencyFallback: false,
            intervened: Math.abs(safetyCorrection) > 1e-12,
            conditioned: Math.abs(conditioningCorrection) > 1e-12,
            correction: safetyCorrection,
            conditioningCorrection,
            reason: Math.abs(safetyCorrection) > 1e-12
              ? 'safety-envelope-limited-command'
              : Math.abs(conditioningCorrection) > 1e-12
                ? 'actuator-plan-conditioned-command'
                : 'proposal-admissible',
            interval,
          };
        } else {
          governor = applySafetyGovernor({
            state: controllerState,
            proposedU: pidConditioned,
            previousU,
            lastMpcSafeU,
            continuationSequence: lastMpcPlan,
            cfg: controlCfg,
          });
          governor = {
            ...governor,
            intervened: true,
            conditioned: Math.abs(conditioningCorrection) > 1e-12,
            correction: governor.u - pidConditioned,
            conditioningCorrection,
          };
          u = governor.u;
        }
      } else {
        u = pid.update(reference, controllerState.x);
      }
    }

    const disturbance = disturbanceAt(t, cfg);
    const equivalentD = equivalentDisturbance(state, u, disturbance, cfg, truthCfg);
    samples.push({
      t,
      x: state.x,
      v: state.v,
      controllerX: controllerState.x,
      controllerV: controllerState.v,
      u,
      reference,
      disturbance,
      equivalentDisturbance: equivalentD,
      truthPlantMismatchEnabled: Boolean(cfg.truthPlant.enabled),
      measurement: measurementSample.value,
      measurementTruth: measurementSample.truth,
      measurementNoise: measurementSample.noise,
      estimationEnabled,
      estimateX: estimationEnabled ? controllerState.x : null,
      estimateV: estimationEnabled ? controllerState.v : null,
      disturbanceEstimateEnabled: disturbanceStateEnabled,
      estimateD: disturbanceStateEnabled ? disturbanceEstimate : null,
      disturbanceVariance: disturbanceStateEnabled ? disturbanceVariance : null,
      innovation: estimationEnabled ? estimatorDiagnostics?.innovation ?? null : null,
      innovationVariance: estimationEnabled ? estimatorDiagnostics?.innovationVariance ?? null : null,
      covarianceTrace: estimationEnabled ? covarianceTrace : null,
      uncertaintyTighteningEnabled: tightening.enabled,
      uncertaintySigmaMultiplier: tightening.sigmaMultiplier,
      uncertaintyPositionSigma: tightening.positionSigma,
      uncertaintyVelocitySigma: tightening.velocitySigma,
      uncertaintyOutputSigma: tightening.outputSigma,
      uncertaintyPositionMargin: tightening.positionMargin,
      uncertaintyVelocityMargin: tightening.velocityMargin,
      uncertaintyOutputMargin: tightening.outputMargin,
      uncertaintyEnvelopeValid: tightening.validEnvelope,
      predictionError: event.predictionError,
      stateChange: event.stateChange,
      constraintRatio: event.constraintRatio,
      triggered,
      triggerReason,
      mpcCost,
      solverStatus,
      fallbackUsed,
      solverConverged: solverDiagnostics?.converged ?? null,
      solverIterations: solverDiagnostics?.iterations ?? null,
      stationarityResidual: solverDiagnostics?.projectedGradientResidual ?? solverDiagnostics?.kktResidual ?? null,
      feasibilityViolation: solverDiagnostics?.feasibilityViolation ?? null,
      activeConstraintRatio: solverDiagnostics?.activeConstraintRatio ?? null,
      safetyViolation: safetyViolationAt(state, cfg),
      pidRaw,
      pidNominal,
      pidConditioned,
      governorEnabled: governorMode,
      governorFeasible: governor?.feasible ?? null,
      governorIntervened: governor?.intervened ?? false,
      governorSafetyIntervened: governor?.intervened ?? false,
      governorConditioned: governor?.conditioned ?? false,
      governorCorrection: governor?.correction ?? 0,
      governorConditioningCorrection: governor?.conditioningCorrection ?? 0,
      governorEmergencyFallback: governor?.emergencyFallback ?? false,
      governorReason: governor?.reason ?? null,
      governorContinuationUsed: governor?.interval?.continuationUsed ?? false,
      governorIntervalLower: governor?.interval?.lower ?? null,
      governorIntervalUpper: governor?.interval?.upper ?? null,
      governorIntervalWidth: governor?.interval?.feasible && Number.isFinite(governor.interval.lower) && Number.isFinite(governor.interval.upper)
        ? governor.interval.upper - governor.interval.lower
        : null,
    });

    expectedState = stepSecondOrderPlant(controllerState, u, 0, controlCfg);
    previousU = u;
    state = stepSecondOrderPlant(state, u, disturbance, truthCfg);

    measurementSample = sensor.read(state);
    if (estimationEnabled) {
      const estimate = estimator.step(u, measurementSample.value);
      controllerState = { x: estimate.x, v: estimate.v };
      disturbanceEstimate = disturbanceStateEnabled ? estimate.d : 0;
      estimatorDiagnostics = estimate.diagnostics;
      estimatorCovariance = estimate.covariance;
      covarianceTrace = estimate.diagnostics.covarianceTrace;
      disturbanceVariance = disturbanceStateEnabled ? estimatorCovariance?.[2]?.[2] ?? null : null;
    } else {
      controllerState = { ...state };
      disturbanceEstimate = 0;
      estimatorDiagnostics = null;
      estimatorCovariance = null;
      covarianceTrace = null;
      disturbanceVariance = null;
    }

    shiftMpcPlan();
  }

  return {
    samples,
    metrics: metrics(samples, cfg.setpoint, solverRecords, cfg),
    config: cfg,
    model,
    truthModel,
    solverRecords,
  };
}

export function compareControllers(config = {}) {
  return ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'].map((mode) => ({ mode, ...runSimulation(mode, config) }));
}
