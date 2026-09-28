import { mergeSimulationConfig } from '../orchestration/simulationConfig.js';

export const EXPERIMENT_SCHEMA = 'mpc-pid-experiment/v1';

export function createExperimentPayload(config, metadata = {}) {
  return {
    schema: EXPERIMENT_SCHEMA,
    createdAt: new Date().toISOString(),
    name: metadata.name || 'MPC PID experiment',
    presetId: metadata.presetId || 'custom',
    notes: metadata.notes || '',
    config: mergeSimulationConfig(config),
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
    config: mergeSimulationConfig(payload.config),
  };
}

export function serializeExperiment(config, metadata = {}) {
  return JSON.stringify(createExperimentPayload(config, metadata), null, 2);
}
