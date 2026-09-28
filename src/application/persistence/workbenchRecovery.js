import { mergeSimulationConfig } from '../../core/orchestration/simulationConfig.js';

export const WORKBENCH_RECOVERY_SCHEMA = 'mpc-pid-workbench-recovery/v1';
export const WORKBENCH_RECOVERY_KEY = 'mpc-pid-system:workbench-recovery';

const MODES = new Set(['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE']);

function storageAvailable() {
  return typeof localStorage !== 'undefined';
}

export function loadWorkbenchRecovery() {
  if (!storageAvailable()) return null;
  try {
    const raw = localStorage.getItem(WORKBENCH_RECOVERY_KEY);
    if (!raw) return null;
    const payload = JSON.parse(raw);
    if (payload?.schema !== WORKBENCH_RECOVERY_SCHEMA) return null;
    return {
      draftCfg: mergeSimulationConfig(payload.draftCfg || {}),
      runCfg: mergeSimulationConfig(payload.runCfg || {}),
      presetId: typeof payload.presetId === 'string' ? payload.presetId : 'baseline',
      activeMode: MODES.has(payload.activeMode) ? payload.activeMode : 'HYBRID_SAFE',
      savedAt: payload.savedAt || null,
    };
  } catch {
    try {
      localStorage.removeItem(WORKBENCH_RECOVERY_KEY);
    } catch {
      // Recovery must never prevent the local web-app from opening.
    }
    return null;
  }
}

export function saveWorkbenchRecovery({ draftCfg, runCfg, presetId, activeMode }) {
  if (!storageAvailable()) return false;
  try {
    localStorage.setItem(WORKBENCH_RECOVERY_KEY, JSON.stringify({
      schema: WORKBENCH_RECOVERY_SCHEMA,
      savedAt: new Date().toISOString(),
      draftCfg: mergeSimulationConfig(draftCfg || {}),
      runCfg: mergeSimulationConfig(runCfg || {}),
      presetId: typeof presetId === 'string' ? presetId : 'baseline',
      activeMode: MODES.has(activeMode) ? activeMode : 'HYBRID_SAFE',
    }));
    return true;
  } catch {
    return false;
  }
}

export function clearWorkbenchRecovery() {
  if (!storageAvailable()) return false;
  try {
    localStorage.removeItem(WORKBENCH_RECOVERY_KEY);
    return true;
  } catch {
    return false;
  }
}
