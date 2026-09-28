import { createUgvClassicalController } from '../controllers/ugvClassicalController.js';
import { createUgvExtendedKalmanFilter } from '../estimation/ugvExtendedKalmanFilter.js';
import {
  createUgvInitialState,
  normalizeAngle,
  stepKinematicBicycle,
} from '../models/kinematicBicycle.js';
import {
  ugvReferenceAtX,
  ugvSafetyViolation,
  ugvTrackingErrors,
} from '../models/ugvReferencePath.js';
import { solveUgvLtvMpc } from '../mpc/ugvLtvMpc.js';
import { computeUgvMetrics } from './ugvMetrics.js';
import { mergeUgvConfig } from './ugvSimulationConfig.js';

export const UGV_CONTROLLER_MODES = Object.freeze({
  CLASSICAL: 'CLASSICAL',
  LTV_MPC: 'LTV_MPC',
});

function solverRecord(solution, t) {
  return {
    t,
    status: solution.status,
    solveMs: solution.solveMs,
    fallbackUsed: solution.fallbackUsed,
    objective: solution.objective,
    diagnostics: solution.diagnostics,
  };
}

export function runUgvSimulation(mode, userConfig = {}) {
  const cfg = mergeUgvConfig(userConfig);
  if (!Object.values(UGV_CONTROLLER_MODES).includes(mode)) {
    throw new Error(`Unsupported UGV controller mode: ${mode}`);
  }

  const steps = Math.floor(cfg.duration / cfg.dt);
  let truth = createUgvInitialState(userConfig.initialState);
  const classical = createUgvClassicalController(cfg);
  const fallbackClassical = createUgvClassicalController(cfg);
  const estimationEnabled = Boolean(cfg.ugvEstimation.enabled);
  const ekf = estimationEnabled
    ? createUgvExtendedKalmanFilter(cfg, truth)
    : null;
  let estimateSnapshot = ekf?.initialize(truth) ?? null;
  let controllerState = estimateSnapshot?.state ?? { ...truth };
  let previousCommand = { acceleration: 0, steering: 0 };
  let appliedCommand = { ...previousCommand };
  let warmStart = null;
  let lastSolveStep = -Infinity;
  const solveEverySteps = Math.max(1, Math.round(cfg.ugvLtvMpc.solveInterval / cfg.dt));
  const samples = [];
  const solverRecords = [];

  for (let k = 0; k <= steps; k += 1) {
    const t = k * cfg.dt;
    const classicalShadow = fallbackClassical.update(controllerState);
    let controllerInfo = null;
    let triggered = false;
    let solverStatus = null;
    let fallbackUsed = false;

    if (mode === UGV_CONTROLLER_MODES.CLASSICAL) {
      controllerInfo = classical.update(controllerState);
      appliedCommand = controllerInfo.command;
    } else if (k === 0 || k - lastSolveStep >= solveEverySteps) {
      const solution = solveUgvLtvMpc(
        controllerState,
        previousCommand,
        cfg,
        warmStart,
        Number.isFinite(lastSolveStep) ? Math.max(1, k - lastSolveStep) : 1,
      );
      warmStart = solution.sequence;
      solverRecords.push(solverRecord(solution, t));
      solverStatus = solution.status;
      fallbackUsed = solution.fallbackUsed;
      appliedCommand = fallbackUsed ? classicalShadow.command : solution.command;
      controllerInfo = {
        command: appliedCommand,
        reference: solution.reference,
        error: solution.error,
      };
      lastSolveStep = k;
      triggered = true;
    } else {
      const reference = ugvReferenceAtX(controllerState.x, cfg);
      controllerInfo = {
        command: appliedCommand,
        reference,
        error: ugvTrackingErrors(controllerState, reference),
      };
    }

    const truthSafety = ugvSafetyViolation(truth, cfg);
    const reference = truthSafety.reference;
    const error = truthSafety.error;
    samples.push({
      t,
      x: truth.x,
      y: truth.y,
      yaw: truth.yaw,
      v: truth.v,
      estimateX: estimationEnabled ? controllerState.x : null,
      estimateY: estimationEnabled ? controllerState.y : null,
      estimateYaw: estimationEnabled ? controllerState.yaw : null,
      estimateV: estimationEnabled ? controllerState.v : null,
      estimateYawError: estimationEnabled ? normalizeAngle(controllerState.yaw - truth.yaw) : null,
      estimationEnabled,
      referenceX: reference.x,
      referenceY: reference.y,
      referenceYaw: reference.yaw,
      referenceSpeed: reference.speed,
      lateralError: error.lateral,
      headingError: error.heading,
      speedError: error.speed,
      acceleration: appliedCommand.acceleration,
      steering: appliedCommand.steering,
      controllerLateralError: controllerInfo.error.lateral,
      controllerHeadingError: controllerInfo.error.heading,
      controllerSpeedError: controllerInfo.error.speed,
      triggered,
      solverStatus,
      fallbackUsed,
      safetyViolation: truthSafety.total,
      laneViolation: truthSafety.lane,
      speedViolation: truthSafety.speed,
      covarianceTrace: estimateSnapshot?.covarianceTrace ?? null,
    });

    previousCommand = { ...appliedCommand };
    truth = stepKinematicBicycle(truth, appliedCommand, cfg);
    if (estimationEnabled) {
      estimateSnapshot = ekf.step(appliedCommand, truth);
      controllerState = estimateSnapshot.state;
    } else {
      controllerState = { ...truth };
    }
  }

  return {
    mode,
    config: cfg,
    samples,
    solverRecords,
    metrics: computeUgvMetrics(samples, solverRecords, cfg),
  };
}

export function compareUgvControllers(userConfig = {}) {
  return [
    runUgvSimulation(UGV_CONTROLLER_MODES.CLASSICAL, userConfig),
    runUgvSimulation(UGV_CONTROLLER_MODES.LTV_MPC, userConfig),
  ];
}
