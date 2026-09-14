import { defaultConfig } from '../simulator.js';

export const EXPERIMENT_SCHEMA = 'mpc-pid-experiment/v1';
export const LOCAL_EXPERIMENT_KEY = 'mpc-pid-system:last-experiment';

function mergeConfig(config = {}) {
  return {
    ...defaultConfig,
    ...config,
    plant: { ...defaultConfig.plant, ...(config.plant || {}) },
    pid: { ...defaultConfig.pid, ...(config.pid || {}) },
    mpc: { ...defaultConfig.mpc, ...(config.mpc || {}) },
    safety: { ...defaultConfig.safety, ...(config.safety || {}) },
    trigger: { ...defaultConfig.trigger, ...(config.trigger || {}) },
    disturbance: { ...defaultConfig.disturbance, ...(config.disturbance || {}) },
  };
}

export function createExperimentPayload(config, metadata = {}) {
  return {
    schema: EXPERIMENT_SCHEMA,
    createdAt: new Date().toISOString(),
    name: metadata.name || 'MPC PID experiment',
    presetId: metadata.presetId || 'custom',
    notes: metadata.notes || '',
    config: mergeConfig(config),
  };
}

export function parseExperimentPayload(input) {
  const payload = typeof input === 'string' ? JSON.parse(input) : input;
  if (!payload || typeof payload !== 'object') throw new Error('Experiment payload must be an object.');
  if (payload.schema !== EXPERIMENT_SCHEMA) throw new Error(`Unsupported experiment schema: ${payload.schema || 'missing'}`);
  if (!payload.config || typeof payload.config !== 'object') throw new Error('Experiment payload is missing config.');
  if (!payload.config.mpc || typeof payload.config.mpc !== 'object') throw new Error('Experiment config is missing MPC settings.');
  return {
    ...payload,
    config: mergeConfig(payload.config),
  };
}

export function serializeExperiment(config, metadata = {}) {
  return JSON.stringify(createExperimentPayload(config, metadata), null, 2);
}

export function saveExperimentLocal(config, metadata = {}) {
  if (typeof localStorage === 'undefined') return false;
  localStorage.setItem(LOCAL_EXPERIMENT_KEY, serializeExperiment(config, metadata));
  return true;
}

export function loadExperimentLocal() {
  if (typeof localStorage === 'undefined') return null;
  const raw = localStorage.getItem(LOCAL_EXPERIMENT_KEY);
  return raw ? parseExperimentPayload(raw) : null;
}

export function downloadExperimentJSON(config, metadata = {}) {
  if (typeof document === 'undefined' || typeof URL === 'undefined' || typeof Blob === 'undefined') return false;
  const text = serializeExperiment(config, metadata);
  const blob = new Blob([text], { type: 'application/json;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  const safeName = (metadata.name || 'mpc-pid-experiment').trim().replace(/[^a-z0-9_-]+/gi, '-').replace(/^-|-$/g, '') || 'mpc-pid-experiment';
  anchor.href = url;
  anchor.download = `${safeName}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
  return true;
}
