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
import { solveMPC } from './solvers/index.js';
import { evaluateEventTrigger } from './triggers/eventTrigger.js';
import { defaultConfig, mergeSimulationConfig } from './orchestration/simulationConfig.js';
import { computeSimulationMetrics, safetyViolationAt } from './orchestration/simulationMetrics.js';

export { defaultConfig };
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
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

function safetyViol(cfg);
  const truthCfg = createTruthPlantConfig(cfg);
  const truthModel = createSecondOrderModel(truthCfg);
  const estimationEnabled = Boolean(cfg.estimation.enabled);
  const disturbanceStateEnabled = Boolean(estimationEnabled && cfg.estimation.disturbanceStateEnabled);
  const disturbancePredictionEnabled = Boolean(disturbanceStateEnabled && cfg.estimation.disturbancePredictionEnabled);
  const mpcDisturbanceCompensationEnabled = Boolean(disturbanceStateEnabled && cfg.estimation.mpcDisturbanceCompensationEnabled);
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
          disturbanceRetention: cfg.estimation.disturbanceRetention,
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
    const predictionDisturbance = disturbancePredictionEnabled ? disturbanceEstimate : 0;
    const solverCfg = mpcDisturbanceCompensationEnabled
      ? {
          ...controlCfg,
          runtime: {
            disturbanceEstimate,
            disturbanceRetention: cfg.estimation.disturbanceRetention,
          },
        }
      : controlCfg;
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
      const solution = solveMPC(controllerState, solverCfg.setpoint, previousU, solverCfg, warmStart);
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
        const solution = solveMPC(controllerState, solverCfg.setpoint, previousU, solverCfg, warmStart);
        warmStart = solution.sequence;
        if (!solution.fallbackUsed) {
          reference = predictiveReference(solution, solverCfg.setpoint, solverCfg);
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
      disturbanceRetention: disturbanceStateEnabled ? cfg.estimation.disturbanceRetention : null,
      disturbancePredictionEnabled,
      predictionDisturbance,
      mpcDisturbanceCompensationEnabled,
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

    expectedState = stepSecondOrderPlant(controllerState, u, predictionDisturbance, controlCfg);
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
    metrics: computeSimulationMetrics(samples, cfg.setpoint, solverRecords, cfg),
    config: cfg,
    model,
    truthModel,
    solverRecords,
  };
}

export function compareControllers(config = {}) {
  return ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'].map((mode) => ({ mode, ...runSimulation(mode, config) }));
}
