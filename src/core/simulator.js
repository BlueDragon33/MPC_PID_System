import { createPIDController } from './controllers/pid.js';
import { applyCovarianceConstraintTightening } from './estimation/uncertaintyTightening.js';
import {
  createSecondOrderModel,
  createTruthPlantConfig,
  disturbanceAt,
  equivalentDisturbance,
  stepSecondOrderPlant,
} from './models/secondOrderPlant.js';
import { computeGovernedPidCommand } from './safety/commandSafetyRuntime.js';
import { evaluateEventTrigger } from './triggers/eventTrigger.js';
import { defaultConfig, mergeSimulationConfig } from './orchestration/simulationConfig.js';
import { computeSimulationMetrics, safetyViolationAt } from './orchestration/simulationMetrics.js';
import { createEstimatorRuntime } from './orchestration/estimatorRuntime.js';
import { createMpcPlanRuntime } from './orchestration/mpcPlanRuntime.js';

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

export function runSimulation(mode, userConfig = {}) {
  const cfg = mergeSimulationConfig(userConfig);
  const steps = Math.floor(cfg.duration / cfg.dt);
  const pid = createPIDController(cfg.pid, cfg.dt);
  const model = createSecondOrderModel(cfg);
  const truthCfg = createTruthPlantConfig(cfg);
  const truthModel = createSecondOrderModel(truthCfg);
  let state = { x: 0, v: 0 };
  const estimatorRuntime = createEstimatorRuntime({ cfg, model, initialState: state });
  const {
    estimationEnabled,
    disturbanceStateEnabled,
    disturbancePredictionEnabled,
    mpcDisturbanceCompensationEnabled,
  } = estimatorRuntime.flags;

  let estimatorSnapshot = estimatorRuntime.current();
  let controllerState = estimatorSnapshot.controllerState;
  let measurementSample = estimatorSnapshot.measurementSample;
  let estimatorDiagnostics = estimatorSnapshot.diagnostics;
  let estimatorCovariance = estimatorSnapshot.covariance;
  let covarianceTrace = estimatorSnapshot.covarianceTrace;
  let disturbanceEstimate = estimatorSnapshot.disturbanceEstimate;
  let disturbanceVariance = estimatorSnapshot.disturbanceVariance;

  let expectedState = { ...controllerState };
  let lastSolveState = { ...controllerState };
  let previousU = 0;
  let lastSolve = -Infinity;
  let reference = cfg.setpoint;
  const mpcPlanRuntime = createMpcPlanRuntime();
  const samples = [];
  const hybridMode = mode === 'HYBRID' || mode === 'HYBRID_SAFE';
  const governorMode = mode === 'HYBRID_SAFE';

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
      const solution = mpcPlanRuntime.solve({
        state: controllerState,
        target: solverCfg.setpoint,
        previousU,
        cfg: solverCfg,
      });
      u = solution.u;
      mpcCost = solution.cost;
      solverDiagnostics = solution.diagnostics || null;
      solverStatus = solution.status || null;
      fallbackUsed = Boolean(solution.fallbackUsed);
      triggered = true;
      triggerReason = 'periodic';
    } else if (hybridMode) {
      if (event.triggered) {
        const solution = mpcPlanRuntime.solve({
          state: controllerState,
          target: solverCfg.setpoint,
          previousU,
          cfg: solverCfg,
        });
        if (!solution.fallbackUsed) {
          reference = predictiveReference(solution, solverCfg.setpoint, solverCfg);
        }
        lastSolve = t;
        lastSolveState = { ...controllerState };
        mpcCost = solution.cost;
        solverDiagnostics = solution.diagnostics || null;
        solverStatus = solution.status || null;
        fallbackUsed = Boolean(solution.fallbackUsed);
        triggered = true;
        triggerReason = event.reason;
      }

      if (governorMode) {
        const governed = computeGovernedPidCommand({
          pid,
          reference,
          controllerState,
          previousU,
          cfg: controlCfg,
          continuationSequence: mpcPlanRuntime.getContinuationSequence(),
          lastMpcSafeU: mpcPlanRuntime.getLastSafeU(),
        });
        u = governed.u;
        pidRaw = governed.pidRaw;
        pidNominal = governed.pidNominal;
        pidConditioned = governed.pidConditioned;
        governor = governed.governor;
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

    estimatorSnapshot = estimatorRuntime.step(u, state);
    controllerState = estimatorSnapshot.controllerState;
    measurementSample = estimatorSnapshot.measurementSample;
    estimatorDiagnostics = estimatorSnapshot.diagnostics;
    estimatorCovariance = estimatorSnapshot.covariance;
    covarianceTrace = estimatorSnapshot.covarianceTrace;
    disturbanceEstimate = estimatorSnapshot.disturbanceEstimate;
    disturbanceVariance = estimatorSnapshot.disturbanceVariance;

    mpcPlanRuntime.shiftPlan();
  }

  const solverRecords = mpcPlanRuntime.getSolverRecords();
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
