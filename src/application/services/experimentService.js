import {
  compareExperimentCases,
  runExperimentMatrix,
} from '../../core/experiments/experimentMatrix.js';
import {
  applyExperimentPreset,
  EXPERIMENT_PRESETS,
  getExperimentPreset,
} from '../../core/experiments/presets.js';
import {
  createExperimentPayload,
  EXPERIMENT_SCHEMA,
  parseExperimentPayload,
  serializeExperiment,
} from '../../core/experiments/serialization.js';
import { assertExperimentMatrixContract } from '../../contracts/workbenchContracts.js';
import {
  downloadExperimentJSON,
  loadExperimentLocal,
  LOCAL_EXPERIMENT_KEY,
  saveExperimentLocal,
} from '../persistence/experimentPersistence.js';

export {
  applyExperimentPreset,
  compareExperimentCases,
  createExperimentPayload,
  downloadExperimentJSON,
  EXPERIMENT_PRESETS,
  EXPERIMENT_SCHEMA,
  getExperimentPreset,
  loadExperimentLocal,
  LOCAL_EXPERIMENT_KEY,
  parseExperimentPayload,
  saveExperimentLocal,
  serializeExperiment,
};

export function executeExperimentMatrix(options) {
  return assertExperimentMatrixContract(runExperimentMatrix(options));
}

export const runExperimentMatrixForWorkbench = executeExperimentMatrix;
