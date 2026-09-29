import {
  parseExperimentPayload,
  serializeExperiment,
} from '../../core/experiments/serialization.js';

export const LOCAL_EXPERIMENT_KEY = 'mpc-pid-system:last-experiment';

export function saveExperimentLocal(config, metadata = {}) {
  if (typeof localStorage === 'undefined') return false;
  try {
    localStorage.setItem(LOCAL_EXPERIMENT_KEY, serializeExperiment(config, metadata));
    return true;
  } catch {
    return false;
  }
}

export function loadExperimentLocal() {
  if (typeof localStorage === 'undefined') return null;
  let raw;
  try {
    raw = localStorage.getItem(LOCAL_EXPERIMENT_KEY);
  } catch {
    throw new Error('Local experiment storage is unavailable.');
  }
  return raw ? parseExperimentPayload(raw) : null;
}

export function downloadExperimentJSON(config, metadata = {}) {
  if (typeof document === 'undefined' || typeof URL === 'undefined' || typeof Blob === 'undefined') return false;
  const text = serializeExperiment(config, metadata);
  const blob = new Blob([text], { type: 'application/json;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  const safeName = (metadata.name || 'mpc-pid-experiment')
    .trim()
    .replace(/[^a-z0-9_-]+/gi, '-')
    .replace(/^-|-$/g, '') || 'mpc-pid-experiment';
  anchor.href = url;
  anchor.download = `${safeName}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
  return true;
}
