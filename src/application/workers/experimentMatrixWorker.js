import { runExperimentMatrix } from '../../core/experiments/experimentMatrix.js';

self.onmessage = ({ data }) => {
  if (data?.type !== 'run') return;
  try {
    const result = runExperimentMatrix(data.options);
    self.postMessage({ type: 'success', result });
  } catch (error) {
    self.postMessage({
      type: 'error',
      error: { message: error?.message || 'Experiment matrix failed.' },
    });
  }
};
