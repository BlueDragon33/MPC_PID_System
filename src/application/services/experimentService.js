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

export function createExperimentMatrixWorker() {
  return new Worker(
    new URL('../workers/experimentMatrixWorker.js', import.meta.url),
    { type: 'module', name: 'mpc-pid-experiment-matrix' },
  );
}

export function executeExperimentMatrixInWorker(options, createWorker = createExperimentMatrixWorker) {
  if (typeof createWorker !== 'function') {
    return Promise.reject(new Error('Background experiment execution is unavailable in this browser.'));
  }

  let worker;
  try {
    worker = createWorker();
  } catch (error) {
    return Promise.reject(error);
  }

  return new Promise((resolve, reject) => {
    let settled = false;
    const finish = (callback, value) => {
      if (settled) return;
      settled = true;
      worker.terminate();
      callback(value);
    };

    worker.onmessage = ({ data }) => {
      if (data?.type === 'success') {
        try {
          finish(resolve, assertExperimentMatrixContract(data.result));
        } catch (error) {
          finish(reject, error);
        }
        return;
      }
      const message = data?.error?.message || 'Experiment matrix worker failed.';
      finish(reject, new Error(message));
    };
    worker.onerror = (event) => {
      event?.preventDefault?.();
      finish(reject, new Error(event?.message || 'Experiment matrix worker failed.'));
    };
    try {
      worker.postMessage({ type: 'run', options });
    } catch (error) {
      finish(reject, error);
    }
  });
}

export const runExperimentMatrixForWorkbench = executeExperimentMatrix;
