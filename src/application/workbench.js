import { runAdaptiveModelShadow } from '../core/adaptation/runAdaptiveModelShadow.js';
import {
  compareExperimentCases,
  runExperimentMatrix,
} from '../core/experiments/experimentMatrix.js';
import {
  applyExperimentPreset,
  EXPERIMENT_PRESETS,
  getExperimentPreset,
} from '../core/experiments/presets.js';
import {
  downloadExperimentJSON,
  loadExperimentLocal,
  parseExperimentPayload,
  saveExperimentLocal,
} from '../core/experiments/serialization.js';
import { compareControllers, defaultConfig, runSimulation } from '../core/simulator.js';
import { SOLVER_BACKENDS } from '../core/solvers/index.js';
import {
  assertControllerComparisonContract,
  assertExperimentMatrixContract,
} from '../contracts/workbenchContracts.js';

export {
  applyExperimentPreset,
  compareExperimentCases,
  defaultConfig,
  downloadExperimentJSON,
  EXPERIMENT_PRESETS,
  getExperimentPreset,
  loadExperimentLocal,
  parseExperimentPayload,
  runAdaptiveModelShadow,
  saveExperimentLocal,
  SOLVER_BACKENDS,
};

export function executeControllerComparison(config) {
  return assertControllerComparisonContract(compareControllers(config));
}

export function executeSimulation(mode, config) {
  return runSimulation(mode, config);
}

export function executeExperimentMatrix(options) {
  return assertExperimentMatrixContract(runExperimentMatrix(options));
}

// Compatibility exports during WP01. Presentation must import only from this module.
export const compareControllersForWorkbench = executeControllerComparison;
export const runExperimentMatrixForWorkbench = executeExperimentMatrix;
